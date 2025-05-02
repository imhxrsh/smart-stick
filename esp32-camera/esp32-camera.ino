#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp32cam.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Pins
const int trigPin = 12;   // HC-SR04 Trigger
const int echoPin = 14;   // HC-SR04 Echo
const int buzzerPin = 4;  // Buzzer (GPIO4)

// GPS
HardwareSerial SerialGPS(2);  // RX2=13, TX2=15
TinyGPSPlus gps;

// Thresholds
const int CRASH_DISTANCE_CM = 10;  // Object <10cm = "unsafe"
const char* WIFI_SSID = "esp";
const char* WIFI_PASS = "1234567890";
const char* SERVER_URL = "http://192.168.164.76:5000/update_location";  // Update this!

WebServer server(80);

// Camera resolutions
static auto loRes = esp32cam::Resolution::find(320, 240);
static auto hiRes = esp32cam::Resolution::find(800, 600);

// ================== ULTRASONIC SENSOR ================== //
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // Timeout after 30ms
  if (duration == 0) {
    return 999.0;  // Return large distance if timeout
  }
  return duration * 0.0343 / 2;
}

// ================== CRASH DETECTION ================== //
bool isCrashDetected() {
  float distance = getDistance();
  bool isObjectTooClose = (distance <= CRASH_DISTANCE_CM && distance > 0);

  //Serial.print("Distance: "); Serial.println(distance);

  return isObjectTooClose;
}

// ================== GPS LOCATION ================== //
void getGPSLocation(float& lat, float& lon) {
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    Serial.print("GPS Location: ");
    Serial.print(lat, 6);
    Serial.print(", ");
    Serial.println(lon, 6);
  } else {
    lat = 0.0;  // Default if no GPS fix
    lon = 0.0;
    //Serial.println("No valid GPS data");
  }
}

// ================== HTTP ALERT (JSON) ================== //
void sendCrashAlert(float lat, float lon, bool isUnsafe) {
  WiFiClient client;
  HTTPClient http;
  http.begin(client, SERVER_URL);
  http.addHeader("Content-Type", "application/json");

  String json = "{";
  json += "\"lat\":" + String(lat, 6) + ",";
  json += "\"lon\":" + String(lon, 6) + ",";
  json += "\"status\":\"";
  json += isUnsafe ? "unsafe" : "safe";
  json += "\"}";

  int httpCode = http.POST(json);
  if (httpCode == HTTP_CODE_OK) {
    Serial.println("Alert sent: " + json);
  } else {
    Serial.println("Server error: " + String(httpCode));
  }
  http.end();
}

// ================== CAMERA STREAMING ================== //
void serveJpg() {
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("Capture Fail - Frame is null");
    server.send(503, "", "");
    return;
  }
  //Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(),
    //            static_cast<int>(frame->size()));

  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

void handleJpgLo() {
  //Serial.println("Attempting to set low resolution");
  if (!esp32cam::Camera.changeResolution(loRes)) {
    Serial.println("SET-LO-RES FAIL");
  } else {
    //Serial.println("Low resolution set successfully");
  }
  serveJpg();
}

void handleJpgHi() {
  //Serial.println("Attempting to set high resolution");
  if (!esp32cam::Camera.changeResolution(hiRes)) {
    Serial.println("SET-HI-RES FAIL");
  } else {
    //Serial.println("High resolution set successfully");
  }
  serveJpg();
}

// ================== SETUP ================== //
void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 13, 15);  // GPS UART2

  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  delay(3000);  // Stabilize sensors

  // Initialize camera with detailed debugging
  Serial.println("Initializing camera...");
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(esp32cam::pins::AiThinker);
    cfg.setResolution(hiRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
    if (!ok) {
      Serial.println("Camera initialization failed. Check connections and pins.");
    }
  }

  // Connect WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  // Print camera URLs
  Serial.print("Low resolution URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/cam-lo.jpg");

  Serial.print("High resolution URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/cam-hi.jpg");

  // Test buzzer briefly
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);

  // Start server routes
  server.on("/cam-lo.jpg", handleJpgLo);
  server.on("/cam-hi.jpg", handleJpgHi);
  server.begin();
  Serial.println("HTTP server started");
}

// ================== MAIN LOOP ================== //
void loop() {
  server.handleClient();
  // Read GPS data
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // Check crash status
  bool isUnsafe = isCrashDetected();
  float lat, lon;
  getGPSLocation(lat, lon);

  // Only trigger alert if unsafe for consecutive readings
  static int unsafeCount = 0;
  if (isUnsafe) {
    unsafeCount++;
    Serial.print("Unsafe count: ");
    Serial.println(unsafeCount);
    if (unsafeCount > 7) {  // Require  consecutive unsafe readings
      digitalWrite(buzzerPin, HIGH);
      Serial.println("CRASH DETECTED! Sending alert...");
      sendCrashAlert(lat, lon, true);
      delay(2000);
      unsafeCount = 0;
    }
  } else {
    digitalWrite(buzzerPin, LOW);
    unsafeCount = 0;
  }

  delay(10);
}