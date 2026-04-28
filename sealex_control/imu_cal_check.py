#!/usr/bin/env python3
"""
SEALEX IMU Calibration Check — imu_cal_check.py
================================================
Platform : ROS 2 Humble  (works in VRX simulation AND on real hardware)
Author   : SEALEX Team

PURPOSE
-------
Run this script BEFORE every real-hardware session (and once in simulation
to establish a baseline). It collects IMU data for a fixed window and gives
you a pass/fail report on:

  1. Heading stability   — std-dev of yaw while boat is stationary.
                           A high number means the IMU is noisy or the
                           magnetometer is being disturbed by motors/ESCs.

  2. Heading drift rate  — linear regression slope over the collection window.
                           Any drift > 0.5 deg/s at rest means you need
                           imu0_differential: true in ekf.yaml.

  3. Gyro bias check     — mean angular velocity at rest should be near zero.
                           Large values indicate the IMU needs re-mounting
                           or the EKF needs a longer warm-up period.

  4. KI_PSI suggestion   — based on measured drift, suggests whether to
                           increase the integral gain in the GNC node.

HOW TO USE
----------
  # Terminal 1 — start VRX (or connect real hardware)
  ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

  # Terminal 2 — run the check (boat must be STATIONARY)
  ros2 run sealex_control imu_cal_check
  # or directly:
  python3 imu_cal_check.py

  # Optional: change collection duration
  ros2 run sealex_control imu_cal_check --ros-args -p duration_sec:=30.0

REAL HARDWARE ADDITIONAL STEPS
-------------------------------
After running this script and fixing any heading issues, do the full
magnetometer calibration by rotating the boat slowly through 2 full circles
while recording:

  ros2 bag record /wamv/sensors/imu/imu/data -o imu_cal_bag

Then use the bag to compute hard-iron offsets and update
magnetic_declination_radians in ekf.yaml.

INSTALLATION
------------
Add to sealex_control/setup.py console_scripts:
    'imu_cal_check = sealex_control.imu_cal_check:main',
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# ── Configuration ─────────────────────────────────────────────────────────────
IMU_TOPIC          = '/esp/imu/data'
DEFAULT_DURATION   = 20.0    # seconds to collect data

# Pass/fail thresholds
HEADING_STD_WARN   = 1.0     # degrees  — warn above this
HEADING_STD_FAIL   = 3.0     # degrees  — fail above this
DRIFT_WARN_DEG_S   = 0.2     # deg/s    — warn above this
DRIFT_FAIL_DEG_S   = 0.5     # deg/s    — fail above this
GYRO_BIAS_WARN     = 0.05    # rad/s    — warn above this


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def mean(data: list) -> float:
    return sum(data) / len(data) if data else 0.0


def std_dev(data: list) -> float:
    if len(data) < 2:
        return 0.0
    m = mean(data)
    return math.sqrt(sum((x - m) ** 2 for x in data) / (len(data) - 1))


def linear_slope(times: list, values: list) -> float:
    """Least-squares linear regression slope (value/second)."""
    n = len(times)
    if n < 2:
        return 0.0
    t_mean = mean(times)
    v_mean = mean(values)
    num = sum((times[i] - t_mean) * (values[i] - v_mean) for i in range(n))
    den = sum((times[i] - t_mean) ** 2 for i in range(n))
    return num / den if den != 0 else 0.0


# ── Collector Node ─────────────────────────────────────────────────────────────

class IMUCalibrationChecker(Node):

    def __init__(self, duration_sec: float = DEFAULT_DURATION):
        super().__init__('imu_cal_check')
        self.duration_sec = duration_sec

        # Raw data storage
        self.timestamps   = []   # float seconds from first message
        self.yaw_samples  = []   # radians (unwrapped)
        self.gyro_z       = []   # rad/s  — yaw rate at rest
        self._start_time  = None
        self._yaw_offset  = None
        self._last_yaw    = None
        self._yaw_accum   = 0.0  # unwrapped yaw accumulator
        self._done        = False

        self.sub = self.create_subscription(
            Imu, IMU_TOPIC, self._imu_cb, 50)

        self.get_logger().info(
            f'\n'
            f'SEALEX IMU Calibration Check\n'
            f'────────────────────────────\n'
            f'Topic    : {IMU_TOPIC}\n'
            f'Duration : {self.duration_sec:.0f} s\n'
            f'\n'
            f'>>> KEEP THE BOAT COMPLETELY STATIONARY <<<\n'
        )

    def _imu_cb(self, msg: Imu):
        if self._done:
            return

        now = time.monotonic()

        if self._start_time is None:
            self._start_time = now
            self.get_logger().info('First IMU message received — collecting...')

        elapsed = now - self._start_time

        # Extract yaw
        q = msg.orientation
        yaw_raw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Unwrap yaw (handles the -π/π discontinuity)
        if self._last_yaw is None:
            self._yaw_offset = yaw_raw
            self._yaw_accum  = 0.0
        else:
            delta = wrap_angle(yaw_raw - self._last_yaw)
            self._yaw_accum += delta
        self._last_yaw = yaw_raw

        self.timestamps.append(elapsed)
        self.yaw_samples.append(self._yaw_accum)          # unwrapped, centred at 0
        self.gyro_z.append(msg.angular_velocity.z)

        # Progress bar
        pct = int(elapsed / self.duration_sec * 40)
        bar = '█' * pct + '░' * (40 - pct)
        print(f'\r  [{bar}] {elapsed:.1f}/{self.duration_sec:.0f}s  '
              f'{len(self.yaw_samples)} samples', end='', flush=True)

        if elapsed >= self.duration_sec:
            self._done = True

    def is_done(self) -> bool:
        return self._done

    def report(self):
        """Print the analysis report and return True if all checks pass."""
        print('\n')

        if len(self.yaw_samples) < 10:
            print('ERROR: Not enough samples — is the IMU topic publishing?')
            print(f'       Check: ros2 topic hz {IMU_TOPIC}')
            return False

        # Convert to degrees for readability
        yaw_deg   = [math.degrees(y) for y in self.yaw_samples]
        n         = len(yaw_deg)
        hz        = n / self.duration_sec

        heading_std  = std_dev(yaw_deg)
        drift_deg_s  = linear_slope(self.timestamps, yaw_deg)
        gyro_mean_z  = mean(self.gyro_z)
        gyro_std_z   = std_dev(self.gyro_z)

        # ── Print report ─────────────────────────────────────────────────────
        SEP = '─' * 55
        print(SEP)
        print('  SEALEX IMU Calibration Check — REPORT')
        print(SEP)
        print(f'  Samples collected : {n}  ({hz:.1f} Hz)')
        print(f'  Duration          : {self.duration_sec:.0f} s')
        print()

        all_pass = True

        # 1. Heading stability
        if heading_std < HEADING_STD_WARN:
            status = 'PASS'
        elif heading_std < HEADING_STD_FAIL:
            status = 'WARN'
            all_pass = False
        else:
            status = 'FAIL'
            all_pass = False
        print(f'  [1] Heading std-dev   : {heading_std:6.3f} deg  → {status}')
        if status == 'WARN':
            print('       Tip: Mount IMU further from motors/ESCs.')
        elif status == 'FAIL':
            print('       Action: IMU is too noisy for reliable navigation.')
            print('               Check for vibration, motor interference, or')
            print('               loose IMU mounting.')

        # 2. Heading drift rate
        drift_abs = abs(drift_deg_s)
        if drift_abs < DRIFT_WARN_DEG_S:
            status = 'PASS'
        elif drift_abs < DRIFT_FAIL_DEG_S:
            status = 'WARN'
            all_pass = False
        else:
            status = 'FAIL'
            all_pass = False
        print(f'  [2] Heading drift     : {drift_deg_s:+.4f} deg/s  → {status}')
        if status in ('WARN', 'FAIL'):
            print('       Action: Set  imu0_differential: true  in ekf.yaml')
            print('               and increase KI_PSI in usv_gnc_node.py.')

        # 3. Gyro bias
        gyro_z_dps = abs(gyro_mean_z) * 180.0 / math.pi
        if abs(gyro_mean_z) < GYRO_BIAS_WARN:
            status = 'PASS'
        else:
            status = 'WARN'
            all_pass = False
        print(f'  [3] Gyro-z bias (mean): {gyro_mean_z:+.5f} rad/s  ({gyro_z_dps:.3f} deg/s)  → {status}')
        print(f'      Gyro-z noise (std) : {gyro_std_z:.5f} rad/s')
        if status == 'WARN':
            print('       Tip: Allow EKF 10+ seconds to estimate and remove gyro bias.')
            print('            If bias persists, re-mount IMU or set a longer ekf delay.')

        # 4. KI_PSI suggestion
        print()
        print(f'  [4] KI_PSI suggestion')
        if drift_abs < DRIFT_WARN_DEG_S:
            ki_suggestion = 0.0
            print(f'       Drift is minimal — keep KI_PSI = 0.0 (simulation default).')
        elif drift_abs < DRIFT_FAIL_DEG_S:
            ki_suggestion = 0.05
            print(f'       Moderate drift detected — start with KI_PSI = 0.05.')
            print(f'       Increase in 0.05 steps until heading bias disappears.')
        else:
            ki_suggestion = 0.1
            print(f'       High drift — start with KI_PSI = 0.10, then tune carefully.')
            print(f'       Also set imu0_differential: true in ekf.yaml.')
        print(f'       → Suggested starting value: KI_PSI = {ki_suggestion}')

        print()
        print(SEP)
        if all_pass:
            print('  OVERALL: PASS — IMU is suitable for navigation.')
        else:
            print('  OVERALL: ISSUES DETECTED — address warnings before real-hardware runs.')
        print(SEP)
        print()

        # Quick-paste block for the GNC node
        print('  Copy into usv_gnc_node.py:')
        print(f'    KI_PSI             = {ki_suggestion}')
        print()
        if drift_abs >= DRIFT_WARN_DEG_S:
            print('  Copy into ekf.yaml:')
            print('    imu0_differential: true')
            print()

        return all_pass


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    # Allow duration override via ROS parameter
    node = IMUCalibrationChecker()
    node.declare_parameter('duration_sec', DEFAULT_DURATION)
    duration_value = node.get_parameter('duration_sec').value
    if duration_value is not None:
        node.duration_sec = float(duration_value)

    try:
        while rclpy.ok() and not node.is_done():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print('\nInterrupted.')

    passed = node.report()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
