import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# ── Vision & Inference ───────────────────────────────────────────────
import cv2
import numpy as np
import onnxruntime as ort   # Requirement: pip install onnxruntime

class AIVisionNode(Node):

    # ─────────────────────────────────────────────────────────────────
    # 1.  CONSTANTS
    # ─────────────────────────────────────────────────────────────────
    K                       = 5000
    OIL_CLASSES             = ["oil_spill"]
    SHIP_CLASSES            = ["ship"]

    STOP    = 0
    RIGHT   = 1
    LEFT    = 2
    FORWARD = 3

    FILTER_OFF   = 0
    FILTER_ON    = 1
    OIL_DETECTED = 2

    OIL_STOP_DISTANCE       = 2   # metres — stop & collect
    OBSTACLE_AVOID_DISTANCE = 5   # metres — start avoidance

    # ── Inference settings ───────────────────────────────────────────
    # USE THE ABSOLUTE PATH HERE
    MODEL_PATH   = "/home/usv/sealex_ws/best.onnx" 
    IMG_SIZE     = 640
    CONF_OIL     = 0.55           
    CONF_GENERAL = 0.25
    NMS_IOU      = 0.45

    def __init__(self):
        super().__init__('ai_vision_node')

        # ── ROS2 publishers ──────────────────────────────────────────
        self.motor_pub  = self.create_publisher(Int32, '/cmd_motor',  10)
        self.filter_pub = self.create_publisher(Int32, '/cmd_filter', 10)

        # ── Anti-spam: remember last sent command ────────────────────
        self.last_motor_cmd  = -1
        self.last_filter_cmd = -1

        # ── Load model ───────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO Model (ONNX) from {self.MODEL_PATH}...")

        try:
            self.session = ort.InferenceSession(
                self.MODEL_PATH,
                providers=["CPUExecutionProvider"]  # Best for Raspberry Pi ARM
            )
            self.input_name = self.session.get_inputs()[0].name
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")
            return

        # Class names must match your training order
        self.class_names = ["oil_spill", "ship"]

        # ── Camera ───────────────────────────────────────────────────
        # Switch to your RTSP stream if mediamtx is running:
        # self.cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/stream")
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("❌ Camera Error: Could not open video source")

        # ── 20 Hz timer ──────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("✅ AI Vision Node (ONNX) Running...")

    # ─────────────────────────────────────────────────────────────────
    # 2.  ONNX INFERENCE HELPERS
    # ─────────────────────────────────────────────────────────────────
    def preprocess(self, frame):
        img = cv2.resize(frame, (self.IMG_SIZE, self.IMG_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return img

    def nms(self, boxes, scores, iou_threshold=0.45):
        if len(boxes) == 0: return []
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1, yy1 = np.maximum(x1[i], x1[order[1:]]), np.maximum(y1[i], y1[order[1:]])
            xx2, yy2 = np.minimum(x2[i], x2[order[1:]]), np.minimum(y2[i], y2[order[1:]])
            w, h = np.maximum(0.0, xx2 - xx1), np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            order = order[1:][iou < iou_threshold]
        return keep

    def run_inference(self, frame):
        h_orig, w_orig = frame.shape[:2]
        inp = self.preprocess(frame)
        outputs = self.session.run(None, {self.input_name: inp})
        predictions = np.squeeze(outputs[0]) # [4+num_classes, 8400]
        predictions = predictions.T # [8400, 6]

        boxes_all, scores_all, cls_all = [], [], []
        for pred in predictions:
            cls_scores = pred[4:]
            cls_id = np.argmax(cls_scores)
            conf = cls_scores[cls_id]
            if conf < self.CONF_GENERAL: continue
            
            cx, cy, bw, bh = pred[0], pred[1], pred[2], pred[3]
            sx, sy = w_orig / self.IMG_SIZE, h_orig / self.IMG_SIZE
            x1, y1 = (cx - bw/2) * sx, (cy - bh/2) * sy
            x2, y2 = (cx + bw/2) * sx, (cy + bh/2) * sy
            
            boxes_all.append([x1, y1, x2, y2])
            scores_all.append(conf)
            cls_all.append(cls_id)

        if not boxes_all: return []
        keep = self.nms(np.array(boxes_all), np.array(scores_all), self.NMS_IOU)
        return [{"name": self.class_names[cls_all[i]], "conf": scores_all[i], "box": boxes_all[i]} for i in keep]

    # ─────────────────────────────────────────────────────────────────
    # 3.  LOGIC & LOOP (UNCHANGED DECISION TREE)
    # ─────────────────────────────────────────────────────────────────
    def get_direction(self, obj_center, center_x):
        if obj_center < center_x - 50: return "LEFT"
        if obj_center > center_x + 50: return "RIGHT"
        return "CENTER"

    def send_motor(self, value: int):
        if value == self.last_motor_cmd: return
        msg = Int32(); msg.data = value
        self.motor_pub.publish(msg)
        self.last_motor_cmd = value
        labels = {0:"STOP", 1:"RIGHT", 2:"LEFT", 3:"FORWARD"}
        self.get_logger().info(f"[MOTOR]  → {labels.get(value)}")

    def send_filter(self, value: int):
        if value == self.last_filter_cmd: return
        msg = Int32(); msg.data = value
        self.filter_pub.publish(msg)
        self.last_filter_cmd = value
        labels = {0:"FILTER_OFF", 1:"FILTER_ON", 2:"OIL_DETECTED"}
        self.get_logger().info(f"[FILTER] → {labels.get(value)}")

    def loop(self):
        ret, frame = self.cap.read()
        if not ret: return

        h, w = frame.shape[:2]
        center_x = w // 2
        detections = self.run_inference(frame)

        oil_detected, oil_close, oil_direction = False, False, "CENTER"
        closest_obstacle, closest_distance = None, float('inf')

        for det in detections:
            name, conf, (x1, y1, x2, y2) = det["name"], det["conf"], det["box"]
            area = (x2 - x1) * (y2 - y1)
            if area <= 0: continue
            distance = self.K / (area ** 0.5)
            obj_center = (x1 + x2) / 2

            if name in self.OIL_CLASSES and conf > self.CONF_OIL:
                oil_detected = True
                oil_direction = self.get_direction(obj_center, center_x)
                if distance <= self.OIL_STOP_DISTANCE: oil_close = True
            else:
                if distance < closest_distance:
                    closest_distance = distance
                    closest_obstacle = {"distance": distance, "obj_center": obj_center}

        # Priority Tree
        if oil_close:
            self.send_motor(self.STOP); self.send_filter(self.FILTER_ON)
        elif oil_detected:
            self.send_filter(self.OIL_DETECTED)
            cmd = {"LEFT": self.LEFT, "RIGHT": self.RIGHT, "CENTER": self.FORWARD}
            self.send_motor(cmd[oil_direction])
        elif closest_obstacle and closest_obstacle["distance"] <= self.OBSTACLE_AVOID_DISTANCE:
            self.send_filter(self.FILTER_OFF)
            obs_dir = self.get_direction(closest_obstacle["obj_center"], center_x)
            self.send_motor(self.LEFT if obs_dir == "RIGHT" else self.RIGHT)
        else:
            self.send_motor(self.FORWARD); self.send_filter(self.FILTER_OFF)

def main():
    rclpy.init()
    node = AIVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()