#!/usr/bin/env python3
"""
SEALEX AI Vision Node
=====================
Runs YOLOv8 object detection on a camera feed and publishes Twist commands
to avoid obstacles or navigate towards oil spills.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String # Optional: For publishing UI status
import cv2
from ultralytics import YOLO

# =====================
# Constants
# =====================
K = 5000

OIL_CLASSES = ["oil_spill"]
SHIP_CLASSES = ["ship"]

# Speed Constants for Twist Messages (Adjust these for your boat!)
FORWARD_SPEED = 1.0     # m/s
TURN_EFFORT   = 50.0    # Matching the units expected by your GNC node

OIL_STOP_DISTANCE       = 2
OBSTACLE_AVOID_DISTANCE = 5

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision_node')

        # 1. Create Publishers
        # We publish to /usv/rc_override so the GNC node handles priority
        self.cmd_vel_pub = self.create_publisher(Twist, '/usv/rc_override', 10)
        
        # Optional: Publish status for a dashboard
        self.status_pub = self.create_publisher(String, '/ai/status', 10)

        # 2. Load the YOLO Model
        self.get_logger().info("Loading YOLO Model...")
        # Make sure 'best.pt' is in the same directory, or provide the full path
        self.model = YOLO("best.pt") 
        self.get_logger().info("Model Loaded!")

        # 3. Initialize Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Camera Error: Could not open /dev/video0")
            # You might want to handle this more gracefully, but for now, exit.
            exit()

        # 4. Start the Vision Loop Timer (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.vision_loop)

        self.last_motor_cmd = "FORWARD"
        self.last_filter_cmd = "FILTER_OFF"
        self.get_logger().info("✅ AI Vision Node Running...")

    def get_direction(self, obj_center, center_x):
        if obj_center < center_x - 50:
            return "LEFT"
        elif obj_center > center_x + 50:
            return "RIGHT"
        else:
            return "CENTER"

    def send_motor_command(self, linear_x, angular_z, label):
        """Creates and publishes the Twist message."""
        if label != self.last_motor_cmd:
            self.get_logger().info(f"[MOTOR] → {label}")
            self.last_motor_cmd = label

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def send_filter_command(self, status):
        """Publishes the filter status (Optional)."""
        if status != self.last_filter_cmd:
             self.get_logger().info(f"[FILTER] → {status}")
             self.last_filter_cmd = status
             
             status_msg = String()
             status_msg.data = status
             self.status_pub.publish(status_msg)

    def vision_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        h, w, _ = frame.shape
        center_x = w // 2

        results = self.model(frame, verbose=False) # verbose=False keeps terminal clean

        oil_detected = False
        oil_close = False
        closest_obstacle = None
        closest_distance = float('inf')

        # =====================
        # DETECTION
        # =====================
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                cls = int(box.cls[0])
                name = self.model.names[cls]
                conf = float(box.conf[0])

                area = (x2 - x1) * (y2 - y1)
                if area == 0:
                    continue

                distance = K / (area ** 0.5)
                obj_center = (x1 + x2) / 2

                # OIL
                if name in OIL_CLASSES and conf > 0.4:
                    oil_detected = True
                    oil_direction = self.get_direction(obj_center, center_x)

                    if distance <= OIL_STOP_DISTANCE:
                        oil_close = True

                # OBSTACLES (ship included)
                else:
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_obstacle = {
                            "distance": distance,
                            "obj_center": obj_center
                        }

        # =====================
        # DECISION & PUBLISHING
        # =====================
        if oil_close:
            self.send_motor_command(0.0, 0.0, "STOP")
            self.send_filter_command("FILTER_ON")

        elif oil_detected:
            self.send_filter_command("OIL_DETECTED")
            if oil_direction == "LEFT":
                 # Turn Left: Zero forward speed, positive angular speed
                self.send_motor_command(0.0, TURN_EFFORT, "LEFT")
            elif oil_direction == "RIGHT":
                # Turn Right: Zero forward speed, negative angular speed
                self.send_motor_command(0.0, -TURN_EFFORT, "RIGHT")
            else:
                self.send_motor_command(FORWARD_SPEED, 0.0, "FORWARD")

        elif closest_obstacle and closest_obstacle["distance"] <= OBSTACLE_AVOID_DISTANCE:
            self.send_filter_command("FILTER_OFF")
            direction = self.get_direction(closest_obstacle["obj_center"], center_x)

            if direction == "RIGHT":
                 # Turn Left to avoid
                self.send_motor_command(0.0, TURN_EFFORT, "AVOID LEFT")
            elif direction == "LEFT":
                 # Turn Right to avoid
                self.send_motor_command(0.0, -TURN_EFFORT, "AVOID RIGHT")
            else:
                 # Default avoid right
                self.send_motor_command(0.0, -TURN_EFFORT, "AVOID RIGHT")

        else:
             # Safe to cruise
            self.send_filter_command("FILTER_OFF")
            self.send_motor_command(FORWARD_SPEED, 0.0, "FORWARD")


def main(args=None):
    rclpy.init(args=args)
    node = AIVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AI Vision Node.")
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()