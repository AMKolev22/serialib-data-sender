import time
import cv2
from ultralytics import YOLO
from client import WS_Client

class Detection:

    def __init__(self):
        self.client = WS_Client()
        self.client.start()
        time.sleep(1)
        self.human_present = False
        
        self.model = YOLO("yolov8n.pt")
        print("YOLOv8 model loaded successfully.")
        self.cap = cv2.VideoCapture(0)

        self.target_fps = 30
        self.frame_delay = 1 / self.target_fps

    def publish_image(self):
        start_time = time.time()

        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model(frame)
        detections = results[0].boxes
        
        human_detected = False
        for detection in detections:
            conf = detection.conf[0].item()
            cls = int(detection.cls[0].item())
            if cls == 0 and conf > 0.5:
                human_detected = True
                if not self.human_present:
                    cv2.putText(frame, "Detected Human", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    self.human_present = True
                break

        if human_detected and not self.human_present:
            self.client.ws.send("Detected human")
            self.human_present = True
        elif not human_detected and self.human_present:
            self.client.ws.send("Road clear")
            self.human_present = False

        cv2.imshow("Person Detection", frame)

        elapsed_time = time.time() - start_time
        remaining_time = self.frame_delay - elapsed_time

        if remaining_time > 0:
            time.sleep(remaining_time)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.shutdown()
            return False
        return True

    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()
        print("Resources released and windows closed.")
