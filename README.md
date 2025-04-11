# **VisionGuard: Object Detection for the Visually Impaired**

**VisionGuard** is an assistive technology project that leverages the **ESP32-CAM** and **YOLOv5** to provide real-time object detection with **audio feedback** for visually impaired individuals. The system captures video feed from the ESP32-CAM, processes it using YOLOv5, and announces detected objects (e.g., _"Warning, person detected"_) via text-to-speech.

## **Project Structure**

```
VisionGuard/
├── detector/
│   └── yolov5.py          # YOLOv5 object detection + audio feedback
├── esp32-camera/          # Arduino IDE code for ESP32-CAM
│   └── (ESP32-CAM code files)
└── ESP32-Library.zip      # Required ESP32-CAM library
```

## **Hardware Requirements**

- **ESP32-CAM module**
- **FTDI Programmer** (for uploading code)
- **Wi-Fi network** (for communication between ESP32 and laptop)

## **Setup & Installation**

### **1. Setting Up ESP32-CAM**

#### **Step 1: Install ESP32 Library in Arduino IDE**

1. Open **Arduino IDE** → **Sketch** → **Include Library** → **Add .ZIP Library**.
2. Select `ESP32-Library.zip` and install.
3. Restart Arduino IDE.

#### **Step 2: Upload Code to ESP32-CAM**

1. Open the `esp32-camera` code in Arduino IDE.
2. Modify **Wi-Fi credentials** (`ssid` & `password`) in the code.
3. Connect ESP32-CAM to your computer via **FTDI programmer**.
4. Select board: **AI Thinker ESP32-CAM** (Tools → Board → ESP32 → AI Thinker ESP32-CAM).
5. Upload the code.

#### **Step 3: Connect to ESP32-CAM Stream**

1. After uploading, open the **Serial Monitor** (Baud rate: **115200**).
2. Press the **RST button** on the ESP32-CAM.
3. Note the **IP address** displayed in the Serial Monitor (e.g., `192.168.1.100`).

### **2. Running YOLOv5 Object Detection with Audio Feedback**

#### **Step 4: Configure `yolov5.py`**

1. Open `detector/yolov5.py`.
2. Locate the `url` variable (e.g., `url = 'http://192.168.1.100/cam-hi.jpg'`).
3. Replace the IP with the one noted from the Serial Monitor.

#### **Step 5: Install Python Dependencies**

```bash
pip install opencv-python numpy torch torchvision pyttsx3
```

#### **Step 6: Run the Detection Script**

```bash
python detector/yolov5.py
```

- The script will:
  - Fetch the video stream from ESP32-CAM.
  - Detect objects in real-time using YOLOv5.
  - Announce warnings via audio (e.g., _"Warning, chair detected"_).

## **How It Works**

1. **ESP32-CAM** captures live video and streams it over Wi-Fi.
2. **YOLOv5** processes the stream and identifies objects.
3. **Audio Feedback**: Uses `pyttsx3` to vocalize detected objects.

## **Future Enhancements**

- **Haptic Feedback**: Vibrations for silent alerts.
- **Edge Deployment**: Run YOLOv5 directly on ESP32 for offline use.
- **Custom Object Training**: Add support for user-specific objects (e.g., keys, doors).

## **Contributing**

Contributions are welcome! Fork the repo, open issues, or submit PRs.

## License

[MIT](./LICENSE)
