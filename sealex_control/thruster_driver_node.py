#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import pigpio  # Hardware PWM library for Raspberry Pi

# RASPBERRY PI PINS (Broadcom numbering)
PIN_LEFT  = 12   # Connect Left ESC signal wire here
PIN_RIGHT = 13   # Connect Right ESC signal wire here

class ThrusterDriver(Node):
    def __init__(self):
        super().__init__('thruster_driver_node')
        
        # Connect to the hardware PWM daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal("Cannot connect to pigpiod! Run 'sudo systemctl start pigpiod'")
            exit(1)

        # Initialize ESCs to 1500 (Stop)
        self.pi.set_servo_pulsewidth(PIN_LEFT, 1500)
        self.pi.set_servo_pulsewidth(PIN_RIGHT, 1500)
        self.get_logger().info("ESCs Initialized and Armed at 1500µs (STOP)")

        # Subscribers listening to your GNC node
        self.sub_left = self.create_subscription(
            Float64, '/wamv/thrusters/left/thrust', self.left_cb, 10)
        self.sub_right = self.create_subscription(
            Float64, '/wamv/thrusters/right/thrust', self.right_cb, 10)

    def convert_thrust_to_pwm(self, thrust_percentage):
        """Map -100/100 to 1100/1900 PWM"""
        # Clamp just in case
        thrust = max(-100.0, min(100.0, thrust_percentage))
        # Linear map: y = mx + b  ->  pwm = 4 * thrust + 1500
        pwm = int(4.0 * thrust + 1500)
        return max(1100, min(1900, pwm))

    def left_cb(self, msg):
        pwm = self.convert_thrust_to_pwm(msg.data)
        self.pi.set_servo_pulsewidth(PIN_LEFT, pwm)

    def right_cb(self, msg):
        pwm = self.convert_thrust_to_pwm(msg.data)
        self.pi.set_servo_pulsewidth(PIN_RIGHT, pwm)

    def stop_motors(self):
        self.pi.set_servo_pulsewidth(PIN_LEFT, 1500)
        self.pi.set_servo_pulsewidth(PIN_RIGHT, 1500)
        self.pi.stop()

def main():
    rclpy.init()
    node = ThrusterDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Shutting down hardware PWM...")
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()