#!/usr/bin/env python3
"""
fake_gps_node.py  —  SEALEX Indoor Demo
========================================
Publishes a fixed NavSatFix on /esp/gps/fix at 10 Hz.
Use this when the NEO-M8M has no satellite signal indoors.

HOW TO USE
----------
1. Set DEMO_LAT and DEMO_LON to the coordinates of your demo hall.
   Find them: open Google Maps, hold your finger on the floor of
   the room, copy the two numbers that appear.

2. Launch the normal stack first:
     ros2 launch sealex_control gnc_with_ekf.launch.py

3. In a second terminal:
     ros2 run sealex_control fake_gps_node

4. Set origin to the EXACT same coordinates:
     ros2 topic pub --once /usv/origin sensor_msgs/NavSatFix \
       "{latitude: 30.5883, longitude: 32.2654, altitude: 0.0}"

5. Send any target as a small offset from those same coordinates.
   Every 0.00001 degree of latitude ≈ 1.1 metres north.
   Every 0.00001 degree of longitude ≈ 0.9 metres east (at Egypt latitude).

   Example — go 5 metres north:
     ros2 topic pub --once /usv/target sensor_msgs/NavSatFix \
       "{latitude: 30.58835, longitude: 32.2654, altitude: 0.0}"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# ===========================================================================
# SET THESE before the demo
# ===========================================================================
DEMO_LAT = 30.602420    # your hall latitude  — positive = North
DEMO_LON = 32.303988    # your hall longitude — positive = East
DEMO_ALT =  0.0       # altitude, leave at 0
# ===========================================================================

PUBLISH_HZ = 10.0


class FakeGPSNode(Node):

    def __init__(self):
        super().__init__('fake_gps_node')

        self.pub = self.create_publisher(NavSatFix, '/esp/gps/fix', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_fix)

        self.msg = NavSatFix()
        self.msg.header.frame_id = 'gps'
        self.msg.latitude  = DEMO_LAT
        self.msg.longitude = DEMO_LON
        self.msg.altitude  = DEMO_ALT

        self.msg.status.status  = NavSatStatus.STATUS_FIX
        self.msg.status.service = NavSatStatus.SERVICE_GPS

        self.msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.msg.position_covariance[0]   = -1.0

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════╗\n'
            '║       FAKE GPS NODE — ACTIVE             ║\n'
            '║  Publishing on /esp/gps/fix at 10 Hz    ║\n'
            f'║  Lat : {DEMO_LAT:.6f}                   ║\n'
            f'║  Lon : {DEMO_LON:.6f}                   ║\n'
            '╚══════════════════════════════════════════╝'
        )

    def publish_fix(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()