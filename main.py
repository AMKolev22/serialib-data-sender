import time
import cv2
import numpy as np
from ultralytics import YOLO
from client import WS_Client

import time
import cv2
import numpy as np
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
        self.zoom_factor = 0.5  # Zoom out to 50% size

    def detect_lanes(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, black_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        height, width = black_mask.shape
        roi = np.array([[  # Define region of interest
            (0, height),
            (width, height),
            (width // 2 + 100, height // 2),
            (width // 2 - 100, height // 2)
        ]], dtype=np.int32)
        mask = np.zeros_like(black_mask)
        cv2.fillPoly(mask, roi, 255)
        roi_edges = cv2.bitwise_and(black_mask, mask)

        lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=150)

        leftmost_line = None
        rightmost_line = None
        if lines is not None:
            lines = sorted(lines, key=lambda line: min(line[0][0], line[0][2]))
            leftmost_line = lines[0][0]
            rightmost_line = lines[-1][0]

        return leftmost_line, rightmost_line, black_mask, roi_edges

    def compute_lookahead_point(self, leftmost_line, rightmost_line, road_width_cm=20):
        if leftmost_line is None or rightmost_line is None:
            return None

        left_midpoint = ((leftmost_line[0] + leftmost_line[2]) // 2, (leftmost_line[1] + leftmost_line[3]) // 2)
        right_midpoint = ((rightmost_line[0] + rightmost_line[2]) // 2, (rightmost_line[1] + rightmost_line[3]) // 2)

        road_center = ((left_midpoint[0] + right_midpoint[0]) // 2, (left_midpoint[1] + right_midpoint[1]) // 2)

        lookahead_distance_px = 100
        lookahead_point = (road_center[0], road_center[1] - lookahead_distance_px)

        return lookahead_point

    def send_lookahead_point(self, point):
        if point:
            lookahead_str = f"{point[0]},{point[1]}"
            # self.client.ws.send(lookahead_str)

    def zoom_out_frame(self, frame):
        new_width = int(frame.shape[1] * self.zoom_factor)
        new_height = int(frame.shape[0] * self.zoom_factor)
        resized_frame = cv2.resize(frame, (new_width, new_height))

        # Add padding to maintain original resolution
        top_pad = (frame.shape[0] - new_height) // 2
        bottom_pad = frame.shape[0] - new_height - top_pad
        left_pad = (frame.shape[1] - new_width) // 2
        right_pad = frame.shape[1] - new_width - left_pad

        padded_frame = cv2.copyMakeBorder(
            resized_frame, top_pad, bottom_pad, left_pad, right_pad, cv2.BORDER_CONSTANT, value=[0, 0, 0]
        )
        return padded_frame

    def publish_image(self):
        start_time = time.time()

        ret, frame = self.cap.read()
        if not ret:
            return False

        # Person detection
        results = self.model(frame)
        detections = results[0].boxes

        human_detected = False
        for detection in detections:
            conf = detection.conf[0].item()
            cls = int(detection.cls[0].item())
            print(f"Class: {cls}, Confidence: {conf}")  # Debug logging
            if cls == 0 and conf > 0.5:
                human_detected = True
                break

        if human_detected and not self.human_present:
            self.client.ws.send("Detected human")
            self.human_present = True
        elif not human_detected and self.human_present:
            self.client.ws.send("Road clear")
            self.human_present = False

        # Display detection result
        cv2.putText(frame, "Human Detected" if human_detected else "Road Clear", 
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
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


if __name__ == "__main__":
    detection = Detection()
    print("Starting the camera feed. Press 'q' to quit.")

    try:
        while True:
            if not detection.publish_image():
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        detection.shutdown()
        print("Application terminated.")
