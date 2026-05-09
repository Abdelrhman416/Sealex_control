#!/usr/bin/env python3
"""
SEALEX GNC Node — Level 5 (Hardware Deployment)
====================================================
Platform : Real Hardware (Raspberry Pi 4 / ESP32) / ROS 2 Humble
Author   : SEALEX Team

Upgrade History
---------------
L1  True IMU heading           quaternion → yaw
L2  Dual-sensor watchdog       GPS + IMU timeouts, one-shot logging
L3  EKF integration            PI closed-loop speed via /odometry/filtered
    Hardware E-STOP            /usv/hw_estop separate from SW E-STOP
    RC Override                /usv/rc_override with auto-timeout
    Thruster calibration       deadband, per-side trim, nonlinear curve
L4  PID heading controller     LOS Guidance, Multi-WP Queue
L5  (Current Version)
    ─────────────────────────────────────────────────────────────────────
    Hardware Integration       Sensors migrated to /esp/... topics
    Pivot Turn Fix             Instantly clears PI speed integrator on 180° turns
    Parking Mode               Soft pause without killing the mission (/usv/park)
    Failsafe Heartbeat         RTL to origin if dashboard disconnects for 10 mins

Topics subscribed
-----------------
  /esp/gps/fix                 sensor_msgs/NavSatFix      (From ESP Microcontroller)
  /esp/imu/data                sensor_msgs/Imu            (From ESP Microcontroller)
  /odometry/filtered           nav_msgs/Odometry          (robot_localization)
  /usv/origin                  sensor_msgs/NavSatFix      (Home point)
  /usv/target                  sensor_msgs/NavSatFix      (Immediate WP)
  /usv/queue_add               sensor_msgs/NavSatFix      (Append WP)
  /usv/mission_start           std_msgs/Bool              (Start/Clear queue)
  /usv/estop                   std_msgs/Bool              (Hard Kill)
  /usv/hw_estop                std_msgs/Bool              (Physical Kill Switch)
  /usv/rc_override             geometry_msgs/Twist        (Manual control)
  /usv/park                    std_msgs/Bool              (Soft Pause)
  /usv/heartbeat               std_msgs/Bool              (Dashboard connection ping)

Topics published
----------------
    /esp/cmd_vel                 geometry_msgs/Twist        (To ESP Microcontroller)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, Int32
from geometry_msgs.msg import Twist

# ===========================================================================
# Physical / world constants
# ===========================================================================
EARTH_RADIUS = 6_371_000.0      # metres

# ===========================================================================
# Watchdog timeouts  (seconds)
# ===========================================================================
GPS_TIMEOUT_SEC = 1.0
IMU_TIMEOUT_SEC = 0.5
EKF_TIMEOUT_SEC = 0.5           # if EKF goes silent, revert to open-loop speed
RC_TIMEOUT_SEC  = 0.5           # RC override auto-clears after this silence
HEARTBEAT_TIMEOUT = 600.0       # 10 minutes failsafe (seconds)

# ===========================================================================
# Slow-zone / alignment guidance parameters
# ===========================================================================
SLOW_ZONE_M      = 8.0                  # distance at which deceleration begins
MIN_SPEED        = 0.3                  # minimum forward speed inside slow zone
ALIGN_THRESHOLD  = math.radians(8)    # heading error → cut speed
ALIGN_HYSTERESIS = math.radians(20)   # cut speed again if error exceeds this
LOS_DELTA_M      = 8.0                # LOS lookahead distance (metres)

MAX_THRUSTER_N     = 100.0  # absolute clamp on final output

# ===========================================================================
# Open-loop fallback gain (used when EKF is not running)
# ===========================================================================
OPEN_LOOP_GAIN = 45.0   # maps m/s command → thrust units  (empirical)

# ===========================================================================
# Closed-loop speed controller gains (active when EKF is running)
# ===========================================================================
KP_V = 40.0     # proportional gain  (surge speed error → thrust)
KI_V =  5.0     # integral gain
V_INTEGRAL_LIMIT = 3.0  # anti-windup clamp on integral accumulator

# ===========================================================================
# PID heading controller gains
# ===========================================================================
KP_PSI             = 1.2
KD_PSI             = 1.0
KI_PSI             = 0.0   # Tune on real hardware to fix wind bias
PSI_INTEGRAL_LIMIT = 0.5   # anti-windup clamp on heading integral

# ===========================================================================
# Pure functions
# ===========================================================================

def gps_to_xy(lat: float, lon: float,
              origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """Flat-earth approximation. Valid for distances < 10 km."""
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    x = d_lon * EARTH_RADIUS * math.cos(math.radians(origin_lat))
    y = d_lat * EARTH_RADIUS
    return x, y


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))

# ===========================================================================
# GNC Node
# ===========================================================================

class USVGNCNode(Node):

    def __init__(self):
        super().__init__('usv_gnc_node')

        # ── Subscribers ────────────────────────────────────────────────────

        # Raw GPS — always subscribed for position (and as EKF fallback)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/esp/gps/fix',
            self.gps_callback, 10)
        
        # ── [NEW] Skimmer Relay ────────────────────────────────────────────
        # Listen to the AI Vision node
        self.ai_filter_sub = self.create_subscription(
            Int32, '/cmd_filter', self.ai_filter_callback, 10)
            
        # Send command to the ESP32 / STM32
        self.skimmer_pub = self.create_publisher(Int32, '/esp/skimmer_cmd', 10)

        # Raw IMU — always subscribed for heading
        self.imu_sub = self.create_subscription(
            Imu, '/esp/imu/data',
            self.imu_callback, 10)

        # ── [L3] EKF filtered odometry ─────────────────────────────────────
        # Published by robot_localization ekf_node.
        # Used for velocity feedback (closed-loop speed controller).
        # If this topic is silent, the node falls back to open-loop thrust.
        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.ekf_callback, 10)

        # Origin lock (one-way door)
        self.origin_sub = self.create_subscription(
            NavSatFix, '/usv/origin',
            self.origin_callback, 10)

        # [L4] Queue control
        self.create_subscription(
            NavSatFix, '/usv/queue_add',
            self.queue_add_callback, 10)
            
        self.create_subscription(
            Bool, '/usv/mission_start',
            self.mission_start_callback, 10)
        
        
        # Dynamic target from web dashboard
        self.target_sub = self.create_subscription(
            NavSatFix, '/usv/target',
            self.target_callback, 10)

        # ── [NEW] Software E-STOP ──────────────────────────────────────────
        # std_msgs/Bool — True engages, False clears.
        # Published by the web dashboard or any ROS node.
        self.sw_estop_sub = self.create_subscription(
            Bool, '/usv/estop',
            self.sw_estop_callback, 10)

        # ── [NEW] Hardware E-STOP ──────────────────────────────────────────
        # Separate topic for a physical kill switch, RC channel relay, or
        # a safety monitor node.  Has IDENTICAL effect to SW E-STOP but
        # is tracked independently so you can see WHICH one fired.
        self.hw_estop_sub = self.create_subscription(
            Bool, '/usv/hw_estop',
            self.hw_estop_callback, 10)

        # ── [NEW] RC Override ──────────────────────────────────────────────
        # geometry_msgs/Twist published by an RC bridge node.
        #   linear.x  ∈ [-1, +1]  — forward/reverse throttle
        #   angular.z ∈ [-1, +1]  — yaw rate (left/right)
        # Override is active while messages arrive within RC_TIMEOUT_SEC.
        # It BYPASSES autonomous control completely but CANNOT bypass E-STOP.
        self.rc_sub = self.create_subscription(
            Twist, '/usv/rc_override',
            self.rc_callback, 10)
        
        # ── [L5] Failsafe & Parking state ──────────────────────────────────
        self.park_active        = False
        self.last_heartbeat     = self.get_clock().now()
        self.failsafe_triggered = False

        self.create_subscription(Bool, '/usv/park', self.park_callback, 10)
        self.create_subscription(Bool, '/usv/heartbeat', self.heartbeat_callback, 10)

        #── Publishers ─────────────────────────────────────────────────────
        # Publishing a single Twist message to the ESP via micro-ROS
        self.cmd_vel_pub = self.create_publisher(Twist, '/esp/cmd_vel', 10)

        # ── Control-loop timer ─────────────────────────────────────────────
        self.dt           = 0.1
        self.timer        = self.create_timer(self.dt, self.control_loop)
        self.print_counter = 0

        # ── Raw sensor state ───────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.psi = 0.0          # heading from IMU (radians)

        self.gps_ready = False
        self.imu_ready = False

        self.last_gps_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()

        self._gps_watchdog_active = False
        self._imu_watchdog_active = False

        # ── [L3] EKF state ─────────────────────────────────────────────────
        self.ekf_ready      = False
        self.ekf_vx_world   = 0.0   # velocity in world (odom) frame — east
        self.ekf_vy_world   = 0.0   # velocity in world (odom) frame — north
        self.last_ekf_time  = self.get_clock().now()
        self._ekf_watchdog_active = False

        # PI speed controller state
        self.v_integral = 0.0

        # ── Origin lock ────────────────────────────────────────────────────
        self.origin_set = False
        self.origin_lat = None
        self.origin_lon = None

        # ── Mission state ──────────────────────────────────────────────────
        self.waypoint_queue = []   # List of (x, y) tuples
        self.target_x       = None
        self.target_y       = None
        self.wp_prev_x      = None
        self.wp_prev_y      = None
        self.mission_active = False
        self._is_aligned = False   # hysteresis flag — False = spinning to align, True = aligned and cruising        

        # ── [NEW] E-STOP state (dual: SW + HW) ────────────────────────────
        self.sw_estop_active = False   # triggered by /usv/estop
        self.hw_estop_active = False   # triggered by /usv/hw_estop

        # ── [NEW] RC Override state ────────────────────────────────────────
        self.rc_override_active = False
        self.rc_linear          = 0.0   # scaled forward command [-1, +1]
        self.rc_angular         = 0.0   # scaled yaw command     [-1, +1]
        self.last_rc_time       = None

        # ── Speed / mission params ─────────────────────────────────────────
        self.v_desired = 3.0   # cruise speed m/s
        self.R_accept  = 1.5   # waypoint acceptance radius m

       # ── Heading controller state ───────────────────────────────────────
        self.prev_e           = 0.0
        self.e_integral       = 0.0
        self.max_angular_speed = 0.8   # rad/s

        # ── Thruster limits ────────────────────────────────────────────────
        self.max_linear_speed = 3.6  # Theoretical max speed of WAM-V in m/s (for guidance scaling)
        self.L = 2.0    # ← MEASURE THIS on the real WAM-V (thruster separation, m)

        # ── Start-up banner ────────────────────────────────────────────────
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════╗\n'
            '║   SEALEX GNC Node  —  Level 4 ACTIVE     ║\n'
            '║   EKF: waiting for /odometry/filtered    ║\n'
            '║   Publish /usv/origin to unlock          ║\n'
            '╚══════════════════════════════════════════╝'
        )

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def any_estop(self) -> bool:
        """True if ANY E-STOP (SW or HW) is currently active."""
        return self.sw_estop_active or self.hw_estop_active

    @property
    def ekf_alive(self) -> bool:
        """True if the EKF topic is healthy right now."""
        if not self.ekf_ready:
            return False
        return self._sensor_age(self.last_ekf_time) < EKF_TIMEOUT_SEC

    @property
    def rc_alive(self) -> bool:
        """True if RC override messages are arriving within timeout."""
        if self.last_rc_time is None:
            return False
        return self._sensor_age(self.last_rc_time) < RC_TIMEOUT_SEC

    # =========================================================================
    # Helpers
    # =========================================================================

    def euler_from_quaternion(self, x: float, y: float,
                               z: float, w: float) -> float:
        """Extract yaw (heading) from a quaternion."""
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def _sensor_age(self, last_time) -> float:
        """Seconds since last_time."""
        return (self.get_clock().now() - last_time).nanoseconds / 1e9

    def stop_boat(self) -> None:
        """Publish zero velocity safely (used by E-STOP and watchdog)."""
        try:
            msg = Twist()
            # Twist messages default to 0.0 for all values automatically
            self.cmd_vel_pub.publish(msg)
        except Exception as ex:
            print(f'[stop_boat] Could not publish: {ex}')

    # =========================================================================
    # [L3] Closed-loop surge speed controller (PI)
    # =========================================================================

    def _compute_thrust_for_speed(self, v_command: float) -> float:
        """
        Return the thrust scalar needed to achieve v_command.

        When EKF is alive   → PI closed-loop  (uses measured surge speed).
        When EKF is silent  → open-loop fallback  (v * OPEN_LOOP_GAIN).

        The returned value is in the same units as OPEN_LOOP_GAIN output
        and is passed to the differential-thrust mixing stage.
        """

        # 👇 THE BUG FIX: Instant kill-switch for Pivot Turns 👇
        # If guidance demands 0.0 speed, instantly erase the integrator 
        # memory and cut forward thrust so the boat can spin in place.
        if v_command == 0.0:
            self.v_integral = 0.0
            return 0.0
        
        if self.ekf_alive:
            # Project world-frame velocity onto boat's forward axis (body surge)
            v_actual = (self.ekf_vx_world * math.cos(self.psi) +
                        self.ekf_vy_world * math.sin(self.psi))
    
            v_error = v_command - v_actual

            # Integrate with anti-windup
            if self.ekf_alive:
               v_actual = (self.ekf_vx_world * math.cos(self.psi) +
                           self.ekf_vy_world * math.sin(self.psi))
               v_error = v_command - v_actual

               thrust = KP_V * v_error + KI_V * self.v_integral
               output = clamp(thrust, 0.0, MAX_THRUSTER_N)

               # Anti-windup: only integrate when output is NOT saturated
               if output < MAX_THRUSTER_N:
                   self.v_integral = clamp(
                       self.v_integral + v_error * self.dt,
                       -V_INTEGRAL_LIMIT, V_INTEGRAL_LIMIT)
            return output
        else:
            # Open-loop: reset integrator so there's no bump on EKF reconnect
            self.v_integral = 0.0
            return clamp(v_command * OPEN_LOOP_GAIN, 0.0, MAX_THRUSTER_N)
        
    def _los_heading(self) -> float:
        """Compute LOS desired heading from the current path segment."""
        # Safety check: If we have no target, just maintain current heading
        if self.target_x is None or self.target_y is None:
            return self.psi
        
        # When close to the waypoint, switch to direct bearing.
        # LOS geometry breaks down when distance < lookahead distance.
        dist = math.hypot(self.target_x - self.x, self.target_y - self.y)
        if dist < LOS_DELTA_M or self.wp_prev_x is None or self.wp_prev_y is None:
            return math.atan2(self.target_y - self.y, self.target_x - self.x)
        
        # Fallback: no path segment yet
        if self.wp_prev_x is None or self.wp_prev_y is None:
            return math.atan2(self.target_y - self.y,
                              self.target_x - self.x)

        # Path angle (direction the segment is pointing)
        pi_p = math.atan2(self.target_y - self.wp_prev_y,
                          self.target_x - self.wp_prev_x)

        # Crosstrack error — perpendicular signed distance from the path
        dx   = self.x - self.wp_prev_x
        dy   = self.y - self.wp_prev_y
        e_ct = -dx * math.sin(pi_p) + dy * math.cos(pi_p)

        # LOS correction: steer toward a point LOS_DELTA_M ahead on the path
        return pi_p + math.atan2(-e_ct, LOS_DELTA_M)


    def _pid_heading(self, psi_desired: float) -> tuple[float, float]:
        """
        PID yaw-rate controller. Returns (omega, heading_error).
        """
        e     = wrap_angle(psi_desired - self.psi)
        e_dot = (e - self.prev_e) / self.dt

        self.e_integral = clamp(
            self.e_integral + e * self.dt,
            -PSI_INTEGRAL_LIMIT, PSI_INTEGRAL_LIMIT)

        omega      = KP_PSI * e + KD_PSI * e_dot + KI_PSI * self.e_integral
        self.prev_e = e

        return clamp(omega, -self.max_angular_speed, self.max_angular_speed), e

    # =========================================================================
    # Speed guidance (distance + heading aware)
    # =========================================================================

    def _compute_velocity_command(self, distance: float,
                                   heading_error: float) -> float:
        abs_e = abs(heading_error)

        # Hysteresis alignment state machine:
        # - When _is_aligned is False: spin in place until error < ALIGN_THRESHOLD
        # - Once moving: keep moving until error > ALIGN_HYSTERESIS
        # This prevents the orbit caused by starting forward motion too early.
        if not self._is_aligned:
            if abs_e > ALIGN_THRESHOLD:
                return 0.0          # still spinning to align — no forward motion
            else:
                self._is_aligned = True   # locked on — start moving

        else:  # currently moving forward
            if abs_e > ALIGN_HYSTERESIS:
                self._is_aligned = False  # lost alignment — stop and re-align
                return 0.0

        # Forward speed profile (only reached when aligned)
        if distance >= SLOW_ZONE_M:
            v = self.v_desired
        else:
            t = (distance - self.R_accept) / (SLOW_ZONE_M - self.R_accept)
            v = MIN_SPEED + clamp(t, 0.0, 1.0) * (self.v_desired - MIN_SPEED)

        # Gentle slow-down for small residual heading error (5°–20°)
        if abs_e > math.radians(5):
            penalty = 1.0 - 0.3 * (abs_e - math.radians(5)) / (ALIGN_HYSTERESIS - math.radians(5))
            v *= clamp(penalty, 0.7, 1.0)

        return clamp(v, 0.0, self.max_linear_speed)

    # =========================================================================
    # Watchdog
    # =========================================================================

    # =========================================================================
    # [L4] Waypoint queue helpers
    # =========================================================================

    def _activate_next_waypoint(self) -> bool:
        """
        Pop the next WP from the queue.
        Sets wp_prev to the just-reached WP so LOS continues smoothly.
        Returns True if a new WP was activated, False if queue is empty.
        """
        if not self.waypoint_queue:
            return False

        # Completed WP becomes the new LOS path-start
        self.wp_prev_x = self.target_x
        self.wp_prev_y = self.target_y

        self.target_x, self.target_y = self.waypoint_queue.pop(0)
        
        # Reset integrators for the new segment
        self.prev_e     = 0.0
        self.e_integral = 0.0
        self.v_integral = 0.0
        self._is_aligned = False

        self.get_logger().info(
            f'[QUEUE] Next WP: X={self.target_x:.2f} Y={self.target_y:.2f}  '
            f'({len(self.waypoint_queue)} remaining)')
        return True


    def _sensors_healthy(self) -> bool:
        """Return True only if both GPS and IMU are receiving data."""
        gps_age = self._sensor_age(self.last_gps_time)
        imu_age = self._sensor_age(self.last_imu_time)

        if gps_age >= GPS_TIMEOUT_SEC:
            if not self._gps_watchdog_active:
                self.get_logger().error(
                    f'[WATCHDOG] 🔴 GPS LOST!  Last packet {gps_age:.2f}s ago.')
                self._gps_watchdog_active = True
            return False

        if imu_age >= IMU_TIMEOUT_SEC:
            if not self._imu_watchdog_active:
                self.get_logger().error(
                    f'[WATCHDOG] 🔴 IMU LOST!  Last packet {imu_age:.2f}s ago.')
                self._imu_watchdog_active = True
            return False

        return True

    # =========================================================================
    # Callbacks — E-STOP (SW)
    # =========================================================================

    def sw_estop_callback(self, msg: Bool) -> None:
        if msg.data:
            if not self.sw_estop_active:
                self.sw_estop_active = True
                self.mission_active  = False
                self.stop_boat()
                self.get_logger().error(
                    '\n⬛ [SW-ESTOP]  SOFTWARE E-STOP ENGAGED — MOTORS KILLED.\n')
        else:
            if self.sw_estop_active:
                self.sw_estop_active = False
                self.get_logger().info(
                    '\n▶  [SW-ESTOP]  Software E-STOP cleared. '
                    'Send a new /usv/target to resume.\n')

    # =========================================================================
    # Callbacks — E-STOP (HW)
    # =========================================================================

    def hw_estop_callback(self, msg: Bool) -> None:
        """
        Triggered by a physical kill switch, RC safety channel, or a
        separate safety-monitor node publishing to /usv/hw_estop.

        On real hardware, wire the kill switch to a micro that publishes
        True on this topic when activated, False when released.
        """
        if msg.data:
            if not self.hw_estop_active:
                self.hw_estop_active = True
                self.mission_active  = False
                self.stop_boat()
                self.get_logger().fatal(
                    '\n🔴 [HW-ESTOP]  HARDWARE E-STOP TRIGGERED — MOTORS KILLED.\n'
                    '   Check physical kill switch.\n')
        else:
            if self.hw_estop_active:
                self.hw_estop_active = False
                self.get_logger().warn(
                    '\n⚠  [HW-ESTOP]  Hardware E-STOP cleared. '
                    'Send a new /usv/target to resume.\n')
                

    # =========================================================================
    # Callbacks — AI Vision Skimmer Relay
    # =========================================================================

    def ai_filter_callback(self, msg: Int32) -> None:
        """
        Receives skimmer command from AI Vision node:
        0 = OFF
        1 = ON (Oil is within 2 meters)
        2 = OIL DETECTED (But far away)
        
        We only want to send 0 or 1 to the ESP32 to actually spin the motor.
        """
        esp_msg = Int32()
        
        if msg.data == 1:
            esp_msg.data = 1
            self.get_logger().info('🟢 [SKIMMER] Oil is close! Sending ON command to ESP32.', throttle_duration_sec=2.0)
        else:
            esp_msg.data = 0
            # We don't need to spam the logs when it's off
            
        self.skimmer_pub.publish(esp_msg)

    # =========================================================================
    # Callbacks — RC Override
    # =========================================================================

    def rc_callback(self, msg: Twist) -> None:
        """
        Receive RC manual-override commands.

        Publish test commands from a terminal:
          ros2 topic pub /usv/rc_override geometry_msgs/Twist \
            "{linear: {x: 0.5}, angular: {z: 0.3}}"

        Expected range: linear.x and angular.z both in [-1, +1].
        """
        if self.any_estop:
            # E-STOP has higher priority — silently ignore RC while stopped
            return

        self.rc_linear  = clamp(msg.linear.x,  -1.0, 1.0)
        self.rc_angular = clamp(msg.angular.z, -1.0, 1.0)
        self.last_rc_time = self.get_clock().now()

        if not self.rc_override_active:
            self.rc_override_active = True
            self.get_logger().warn(
                f'\n⚡ [RC-OVERRIDE]  ACTIVE — Autonomous control suspended.\n'
                f'   linear.x={self.rc_linear:+.2f}  angular.z={self.rc_angular:+.2f}\n')

    # =========================================================================
    # Callbacks — Origin, GPS, IMU, EKF, Target
    # =========================================================================

    def origin_callback(self, msg: NavSatFix) -> None:
        if self.origin_set:
            self.get_logger().warn('[ORIGIN] Already locked — ignoring.')
            return
        self.origin_lat = msg.latitude
        self.origin_lon = msg.longitude
        self.origin_set = True
        self.get_logger().info(
            f'[ORIGIN] ✓ LOCKED — '
            f'lat={self.origin_lat:.8f}  lon={self.origin_lon:.8f}')

    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps_time = self.get_clock().now()
        if self._gps_watchdog_active:
            self.get_logger().info('[GPS] ✓ Signal recovered.')
            self._gps_watchdog_active = False
        if not self.origin_set or self.origin_lat is None or self.origin_lon is None:
            return
        self.x, self.y = gps_to_xy(
            msg.latitude, msg.longitude,
            self.origin_lat, self.origin_lon)
        if not self.gps_ready:
            self.gps_ready = True
            self.get_logger().info(
                f'[GPS] ✓ First fix  X={self.x:.3f} m  Y={self.y:.3f} m')

    def imu_callback(self, msg: Imu) -> None:
        self.last_imu_time = self.get_clock().now()
        if self._imu_watchdog_active:
            self.get_logger().info('[IMU] ✓ Signal recovered.')
            self._imu_watchdog_active = False
        q = msg.orientation
        self.psi = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        if not self.imu_ready:
            self.imu_ready = True
            self.get_logger().info(
                f'[IMU] ✓ First reading  Yaw={math.degrees(self.psi):.1f}°')

    def ekf_callback(self, msg: Odometry) -> None:
        """
        Receive filtered odometry from robot_localization ekf_node.

        We use ONLY the velocity (twist) from EKF, not the position.
        Position still comes from raw GPS + flat-earth conversion so it
        stays consistent with the target coordinate system.

        Velocity in the Odometry message is expressed in the child_frame_id
        (typically base_link / body frame) when robot_localization is
        configured with two_d_mode: true.  We store it as world-frame
        equivalent by rotating back — but for 2-D surge the body-forward
        velocity is the number we need regardless of frame.
        """
        # Store world-frame velocities for body-frame projection in the
        # speed controller (psi rotation applied there).
        self.ekf_vx_world = msg.twist.twist.linear.x
        self.ekf_vy_world = msg.twist.twist.linear.y
        self.last_ekf_time = self.get_clock().now()

        if self._ekf_watchdog_active:
            self.get_logger().info('[EKF] ✓ /odometry/filtered recovered — closed-loop speed active.')
            self._ekf_watchdog_active = False

        if not self.ekf_ready:
            self.ekf_ready = True
            self.get_logger().info(
                '[EKF] ✓ First odometry received — '
                'PI speed controller ACTIVE.\n'
                '      Open-loop fallback DISABLED while EKF is alive.')

    # =========================================================================
    # Callbacks — mission / target
    # =========================================================================

    def target_callback(self, msg: NavSatFix) -> None:
        """Single immediate WP — clears queue, backward-compatible."""
        if self.any_estop:
            self.get_logger().warn('[TARGET] E-STOP active — ignoring.')
            return
        if not self.origin_set or self.origin_lat is None or self.origin_lon is None:
            self.get_logger().warn('[TARGET] No origin — publish /usv/origin first.')
            return
            
        self.waypoint_queue.clear()
        self.wp_prev_x = self.x
        self.wp_prev_y = self.y
        self.target_x, self.target_y = gps_to_xy(
            msg.latitude, msg.longitude, self.origin_lat, self.origin_lon)
            
        self.mission_active = True
        self.prev_e         = 0.0
        self.e_integral     = 0.0
        self.v_integral     = 0.0
        self._is_aligned = False
        
        self.get_logger().info(
            f'[TARGET] Immediate WP: X={self.target_x:.3f} Y={self.target_y:.3f}')

    def queue_add_callback(self, msg: NavSatFix) -> None:
        """[L4] Append one WP to the mission queue."""
        if not self.origin_set or self.origin_lat is None or self.origin_lon is None:
            self.get_logger().warn('[QUEUE] No origin — publish /usv/origin first.')
            return
            
        wx, wy = gps_to_xy(
            msg.latitude, msg.longitude, self.origin_lat, self.origin_lon)
        self.waypoint_queue.append((wx, wy))
        
        self.get_logger().info(
            f'[QUEUE] Added WP: X={wx:.2f} Y={wy:.2f}  '
            f'(queue size: {len(self.waypoint_queue)})')

    def mission_start_callback(self, msg: Bool) -> None:
        """[L4] True: start queue.  False: clear queue and stop."""
        if not msg.data:
            self.waypoint_queue.clear()
            self.mission_active = False
            self.target_x = self.target_y = None
            self.wp_prev_x = self.wp_prev_y = None
            self.stop_boat()
            self.get_logger().info('[QUEUE] Mission cleared.')
            return

        if self.any_estop:
            self.get_logger().warn('[QUEUE] E-STOP active — cannot start.')
            return
        if not self.origin_set:
            self.get_logger().warn('[QUEUE] No origin set.')
            return
        if not self.waypoint_queue:
            self.get_logger().warn('[QUEUE] Queue is empty — add waypoints first.')
            return

        self.wp_prev_x = self.x
        self.wp_prev_y = self.y
        self.target_x, self.target_y = self.waypoint_queue.pop(0)
        self.mission_active = True
        self.prev_e         = 0.0
        self.e_integral     = 0.0
        self.v_integral     = 0.0
        self._is_aligned = False
        
        self.get_logger().info(
            f'[QUEUE] Mission STARTED — WP1: X={self.target_x:.2f} Y={self.target_y:.2f}  '
            f'({len(self.waypoint_queue)} more in queue)')
        

    # =========================================================================
    # Callbacks — Failsafe & Parking
    # =========================================================================

    def park_callback(self, msg: Bool) -> None:
        self.park_active = msg.data
        if self.park_active:
            self.get_logger().warn('\n[PARK] Parking Mode ENGAGED — Motors on standby.\n')
            self.stop_boat()
        else:
            self.get_logger().info('\n[PARK] Parking Mode CLEARED — Resuming mission.\n')

    def heartbeat_callback(self, msg: Bool) -> None:
        self.last_heartbeat = self.get_clock().now()
        if self.failsafe_triggered:
            self.get_logger().info('[FAILSAFE] Connection restored!')
            self.failsafe_triggered = False

    # =========================================================================
    # Main control loop  (10 Hz)
    # =========================================================================

    def control_loop(self) -> None:

        # ── Priority 0: Park Mode ──────────────────────────────────────────
        if self.park_active:
            self.stop_boat()
            return

        # ── Priority 1: ANY E-STOP ─────────────────────────────────────────
        if self.any_estop:
            source = []
            if self.sw_estop_active: source.append('SW')
            if self.hw_estop_active: source.append('HW')
            # Keep publishing zeros every tick while E-STOP is active
            # so the thrusters cannot drift on from a stale command.
            self.stop_boat()
            return

        # ── Priority 2: 15-Minute Failsafe (Heartbeat Check) ───────────────
        if self._sensor_age(self.last_heartbeat) > 600.0:  # 600s = 10 mins
            if not self.failsafe_triggered:
                self.get_logger().error(
                    '\n!!! FAILSAFE: 10 MINUTE CONNECTION LOST !!!\n'
                    'Clearing queue and returning to Origin immediately.\n')
                self.failsafe_triggered = True
                self.waypoint_queue.clear()
                if self.origin_set:
                    # In our ENU map, the locked origin is exactly (0.0, 0.0)
                    self.target_x, self.target_y = 0.0, 0.0
                    self.mission_active = True
                else:
                    self.stop_boat()
            # Let it continue to drive to 0,0 or idle if no origin.

        # ── Priority 3: RC Override ────────────────────────────────────────
        if self.rc_alive:
            if not self.rc_override_active:
                self.rc_override_active = True
            self._execute_rc_override()
            return
        else:
            if self.rc_override_active:
                self.rc_override_active = False
                self.stop_boat()
                self.get_logger().info(
                    '[RC-OVERRIDE] Timeout — RC lost. '
                    'Autonomous control restored.')
            # Fall through to autonomous ↓

        # ── Priority 4: Origin lock ────────────────────────────────────────
        if not self.origin_set:
            self.get_logger().warn(
                'Waiting for origin — publish NavSatFix to /usv/origin',
                throttle_duration_sec=5.0)
            return

        # ── Priority 5: Sensor readiness ──────────────────────────────────
        if not self.gps_ready or not self.imu_ready:
            self.get_logger().warn(
                f'Waiting for sensors — '
                f'GPS:{"✓" if self.gps_ready else "✗"} '
                f'IMU:{"✓" if self.imu_ready else "✗"}',
                throttle_duration_sec=2.0)
            return

        # ── Priority 6: Sensor watchdog ────────────────────────────────────
        if not self._sensors_healthy():
            self.stop_boat()
            return

        # ── EKF watchdog (non-blocking — only affects speed source) ───────
        if self.ekf_ready and not self.ekf_alive:
            if not self._ekf_watchdog_active:
                self.get_logger().warn(
                    '[EKF] ⚠  /odometry/filtered lost — '
                    'reverting to open-loop speed control.')
                self._ekf_watchdog_active = True
                self.v_integral = 0.0   # reset integrator

        # ── Priority 6: No active mission ─────────────────────────────────
        if not self.mission_active or self.target_x is None:
            self.get_logger().info(
                f'Sensors OK (EKF:{"✓" if self.ekf_alive else "✗ fallback"}) '
                f'— idling, waiting for /usv/target.',
                throttle_duration_sec=5.0)
            return

        # ── Active autonomous mission ──────────────────────────────────────
        if self.target_x is None or self.target_y is None:
            self.get_logger().warn(
                'Target coordinates are None — skipping mission update.')
            return

        distance = math.hypot(
            self.target_x - self.x,
            self.target_y - self.y)

        # Waypoint acceptance
        if distance < self.R_accept:
            self.get_logger().info(
                f'\n[WP REACHED] ({self.target_x:.2f}, {self.target_y:.2f})  '
                f'dist={distance:.2f}m\n')
                
            if not self._activate_next_waypoint():
                self.get_logger().info(
                    '\n[MISSION COMPLETE] All waypoints reached.\n')
                self.stop_boat()
                self.mission_active = False
                self.target_x = self.target_y = None
                self.wp_prev_x = self.wp_prev_y = None
            return

        ## [L4] LOS desired heading
        psi_desired = self._los_heading()
        
        # PID heading controller
        omega, e = self._pid_heading(psi_desired)

        # Desired speed (guidance)
        v_cmd = self._compute_velocity_command(distance, e)

        # Thrust for that speed (closed-loop PI or open-loop)
        thrust_fwd = self._compute_thrust_for_speed(v_cmd)

        # Scale angular velocity (omega) into a turning effort (matching thrust units)
        yaw_effort = omega * self.L / 2.0 * (MAX_THRUSTER_N / self.max_linear_speed)

        # Create and publish Twist message to the ESP
        msg = Twist()
        msg.linear.x = thrust_fwd
        msg.angular.z = yaw_effort
        self.cmd_vel_pub.publish(msg)

        # ── Telemetry (every 10 ticks = 1 s) ──────────────────────────────
        self.print_counter += 1
        if self.print_counter >= 10:
            zone      = 'SLOW  ' if distance < SLOW_ZONE_M else 'CRUISE'
            align     = 'ALIGN' if abs(e) > ALIGN_THRESHOLD else 'OK   '
            speed_src = 'EKF-PI' if self.ekf_alive else 'OPEN-L'
            self.get_logger().info(
                f'[{zone}|{align}|{speed_src}] '
                f'Tgt:({self.target_x:.1f},{self.target_y:.1f})  '
                f'Pose:X={self.x:.2f} Y={self.y:.2f} Yaw={math.degrees(self.psi):.1f}°  '
                f'Dist={distance:.2f}m  '
                f'vCmd={v_cmd:.2f} Fwd={thrust_fwd:.1f}  '
                f'ω={omega:.2f}  '
                f'YawEffort={yaw_effort:.1f}')
            self.print_counter = 0

    # =========================================================================
    # RC override execution
    # =========================================================================

    def _execute_rc_override(self) -> None:
        """
        Map RC commands to a Twist message for the ESP.
        """
        fwd_thrust = self.rc_linear  * MAX_THRUSTER_N
        yaw_thrust = self.rc_angular * MAX_THRUSTER_N

        msg = Twist()
        msg.linear.x = fwd_thrust
        msg.angular.z = yaw_thrust
        self.cmd_vel_pub.publish(msg)

        self.print_counter += 1
        if self.print_counter >= 10:
            self.get_logger().info(
                f'[RC-OVERRIDE]  fwd={self.rc_linear:+.2f}  '
                f'yaw={self.rc_angular:+.2f}  '
                f'FwdEffort={fwd_thrust:.1f} YawEffort={yaw_thrust:.1f}')
            self.print_counter = 0


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = USVGNCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n>>> DEAD MAN'S SWITCH: Killing Motors! <<<")
        node.stop_boat()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()