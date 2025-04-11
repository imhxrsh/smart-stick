import cv2
import urllib.request
import numpy as np
import torch
import pyttsx3
import threading
import time
from queue import Queue

class ObjectDetector:
    def __init__(self, model_name='yolov5m', conf_threshold=0.3):
        self.model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
        self.model.conf = conf_threshold
        self.hazardous_objects = self.get_hazardous_objects()
        self.tts_engine = self.init_tts()
        self.last_alert_time = 0
        self.alert_cooldown = 3

    def get_hazardous_objects(self):
        """Return optimized list of hazardous objects."""
        return {
            'car', 'motorcycle', 'bicycle', 'truck', 'bus',
            'traffic light', 'fire hydrant', 'stop sign', 'bench',
            'dog', 'cat', 'chair', 'table'
        }

    def init_tts(self):
        """Initialize Text-to-Speech engine in a separate thread."""
        tts_engine = pyttsx3.init()
        tts_engine.setProperty('rate', 150)
        tts_engine.setProperty('volume', 0.8)
        return tts_engine

    def text_to_speech(self, message):
        """Non-blocking text to speech with cooldown."""
        current_time = time.time()
        if current_time - self.last_alert_time > self.alert_cooldown:
            self.last_alert_time = current_time
            threading.Thread(
                target=self._async_tts,
                args=(message,),
                daemon=True
            ).start()

    def _async_tts(self, message):
        """Run TTS in a separate thread."""
        print(f"Alert: {message}")
        self.tts_engine.say(message)
        self.tts_engine.runAndWait()

    def detect_objects(self, frame):
        """Object detection with YOLOv5 using original frame size"""
        results = self.model(frame)
        return results.pandas().xyxy[0]

    def process_frame(self, frame):
        """Frame processing with hazardous object detection only."""
        results = self.detect_objects(frame)
        detected_hazards = set()

        for _, row in results.iterrows():
            x1, y1, x2, y2 = map(int, [row['xmin'], row['ymin'], row['xmax'], row['ymax']])
            label = self.model.names[int(row['class'])]
            conf = row['confidence']

            if label in self.hazardous_objects:
                detected_hazards.add(label)
                color = (0, 0, 255)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.generate_alerts(detected_hazards)
        return frame

    def generate_alerts(self, hazards):
        """Alert generation for hazardous objects only."""
        alerts = []
        
        for hazard in hazards:
            alerts.append(f"{hazard} detected")

        if alerts:
            self.text_to_speech("Warning! " + ". ".join(alerts))

class FrameBuffer:
    """Buffer to handle frame processing in separate threads."""
    def __init__(self, max_size=2):
        self.queue = Queue(maxsize=max_size)
        self.latest_frame = None

    def put(self, frame):
        if not self.queue.full():
            self.queue.put(frame)
        self.latest_frame = frame

    def get(self):
        if not self.queue.empty():
            return self.queue.get()
        return self.latest_frame

def camera_thread(url, frame_buffer):
    """Thread for continuously capturing frames from ESP32 CAM."""
    while True:
        try:
            img_resp = urllib.request.urlopen(url, timeout=2)
            img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            frame = cv2.imdecode(img_np, -1)
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            frame_buffer.put(frame)
        except Exception as e:
            print(f"Camera error: {e}")
            time.sleep(0.1)

def main():
    url = 'http://YOUR_IP/cam-hi.jpg'
    win_name = 'ESP32 CAMERA - YOLOv5 Detection'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    detector = ObjectDetector(model_name='yolov5m', conf_threshold=0.4)
    frame_buffer = FrameBuffer(max_size=2)

    threading.Thread(
        target=camera_thread,
        args=(url, frame_buffer),
        daemon=True
    ).start()

    last_frame_time = time.time()
    fps_counter = 0
    fps = 0

    while True:
        frame = frame_buffer.get()
        if frame is not None:
            processed_frame = detector.process_frame(frame)

            fps_counter += 1
            if time.time() - last_frame_time >= 1.0:
                fps = fps_counter
                fps_counter = 0
                last_frame_time = time.time()

            cv2.putText(processed_frame, f"FPS: {fps}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow(win_name, processed_frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()