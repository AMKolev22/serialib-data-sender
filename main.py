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
        self.zoom_factor = 0.5

        self.last_send_time = 0
        self.last_sent_point = None 
        self.smoothing_window = 5
        self.lookahead_points_history = []  

    def detect_lanes(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        black_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

        height, width = black_mask.shape
        roi = np.array([[  
            (0, height),
            (width, height),
            (width // 2 + 100, height // 2),
            (width // 2 - 100, height // 2)
        ]], dtype=np.int32)
        mask = np.zeros_like(black_mask)
        cv2.fillPoly(mask, roi, 255)
        roi_edges = cv2.bitwise_and(black_mask, mask)

        edges = cv2.Canny(roi_edges, 50, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=150)

        leftmost_line = None
        rightmost_line = None
        if lines is not None:
            lines = [line for line in lines if self._is_valid_slope(line)]
            if lines:
                lines = sorted(lines, key=lambda line: min(line[0][0], line[0][2]))
                leftmost_line = lines[0][0]
                rightmost_line = lines[-1][0]

                leftmost_line = self._fit_polynomial(leftmost_line)
                rightmost_line = self._fit_polynomial(rightmost_line)

        return leftmost_line, rightmost_line, black_mask, edges

    def _is_valid_slope(self, line):

        x1, y1, x2, y2 = line[0]
        if x2 == x1:
            return False  # Avoid division by zero
        slope = (y2 - y1) / (x2 - x1)
        return 0.5 < abs(slope) < 2

    def _fit_polynomial(self, line):

        x1, y1, x2, y2 = line
        poly = np.polyfit([x1, x2], [y1, y2], 1)
        x_new = np.linspace(x1, x2, num=100, dtype=int)
        y_new = np.polyval(poly, x_new).astype(int)

        return [x_new[0], y_new[0], x_new[-1], y_new[-1]]

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
            self.client.ws.send(lookahead_str)
    def smooth_lookahead_point(self, point):
        if len(self.lookahead_points_history) >= self.smoothing_window:
            self.lookahead_points_history.pop(0)

        self.lookahead_points_history.append(point)

        avg_x = int(np.mean([p[0] for p in self.lookahead_points_history]))
        avg_y = int(np.mean([p[1] for p in self.lookahead_points_history]))

        return avg_x, avg_y

    def zoom_out_frame(self, frame):
        new_width = int(frame.shape[1] * self.zoom_factor)
        new_height = int(frame.shape[0] * self.zoom_factor)
        resized_frame = cv2.resize(frame, (new_width, new_height))

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

        results = self.model(frame)
        detections = results[0].boxes

        human_detected = False
        for detection in detections:
            conf = detection.conf[0].item()
            cls = int(detection.cls[0].item())
            if cls == 0 and conf > 0.5:
                human_detected = True
                break

        if human_detected and not self.human_present:
            self.client.ws.send("Detected human")
            self.human_present = True
        elif not human_detected and self.human_present:
            self.client.ws.send("Road clear")
            self.human_present = False

        leftmost_line, rightmost_line, _, _ = self.detect_lanes(frame)

        lookahead_point = self.compute_lookahead_point(leftmost_line, rightmost_line)

        if lookahead_point:
            smoothed_point = self.smooth_lookahead_point(lookahead_point)

            if self.last_sent_point is None or abs(smoothed_point[0] - self.last_sent_point[0]) > 10 or abs(smoothed_point[1] - self.last_sent_point[1]) > 10:
                self.send_lookahead_point(smoothed_point)
                self.last_sent_point = smoothed_point 

        if leftmost_line is not None:
            cv2.line(frame, (leftmost_line[0], leftmost_line[1]), (leftmost_line[2], leftmost_line[3]), (0, 255, 0), 2)
        if rightmost_line is not None:
            cv2.line(frame, (rightmost_line[0], rightmost_line[1]), (rightmost_line[2], rightmost_line[3]), (255, 0, 0), 2)
        if lookahead_point:
            cv2.circle(frame, smoothed_point, 5, (0, 0, 255), -1)

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
