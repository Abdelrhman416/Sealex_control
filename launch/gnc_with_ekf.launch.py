"""
gnc_with_ekf.launch.py
======================
Launches the full SEALEX GNC stack with robot_localization EKF:

    navsat_transform_node  — GPS NavSatFix → local ENU odometry (/odometry/gps)
    ekf_node               — fuses IMU + GPS odometry → /odometry/filtered
    usv_gnc_node           — SEALEX guidance, navigation & control

PREREQUISITE
------------
  sudo apt install ros-humble-robot-localization

PLACE THIS FILE AT
------------------
  sealex_control/launch/gnc_with_ekf.launch.py

USAGE
-----
  # Terminal 1 — VRX simulation (run this first, wait for Gazebo to fully load)
  ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

  # Terminal 2 — GNC + EKF stack
  ros2 launch sealex_control gnc_with_ekf.launch.py

  # Optional: verbose EKF output for debugging
  ros2 launch sealex_control gnc_with_ekf.launch.py ekf_debug:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share  = get_package_share_directory('sealex_control')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # ── Launch arguments ────────────────────────────────────────────────────
    ekf_debug_arg = DeclareLaunchArgument(
        'ekf_debug',
        default_value='false',
        description='Set true to print EKF internals to terminal.'
    )
    ekf_debug = LaunchConfiguration('ekf_debug')

    # ── navsat_transform_node ───────────────────────────────────────────────
    # Converts GPS (NavSatFix, WGS-84) into a local ENU odometry frame so
    # the EKF can treat position as a Cartesian measurement.
    #
    # Topic remappings for VRX 3.1.0 / Gazebo Garden:
    #   gps/fix           ← /wamv/sensors/gps/gps/fix
    #   imu/data          ← /wamv/sensors/imu/imu/data
    #   odometry/filtered ← /odometry/filtered  (EKF feedback, needed for
    #                        the GPS→odom coordinate correction loop)
    #   Output:
    #   odometry/gps      → /odometry/gps       (consumed by ekf_node)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('imu/data',           '/wamv/sensors/imu/imu/data'),
            ('gps/fix',            '/wamv/sensors/gps/gps/fix'),
            ('odometry/filtered',  '/odometry/filtered'),
            ('odometry/gps',       '/odometry/gps'),
        ],
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    # ── ekf_node ────────────────────────────────────────────────────────────
    # Extended Kalman Filter: fuses GPS odometry + IMU → /odometry/filtered
    # The GNC node reads velocity from this topic for its PI speed controller.
    #
    # NOTE ON FRAMES (VRX 3.x):
    #   VRX publishes TF:  wamv/base_link → wamv/imu_wamv_link  (via URDF)
    #   ekf.yaml sets:     base_link_frame: wamv/base_link
    #   If you see TF errors like "could not find transform", run:
    #     ros2 run tf2_tools view_frames
    #   and compare the output to base_link_frame in ekf.yaml.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('imu0',              '/wamv/sensors/imu/imu/data'),
            ('odom0',             '/odometry/gps'),
        ],
    )

    # ── usv_gnc_node ────────────────────────────────────────────────────────
    gnc_node = Node(
        package='sealex_control',
        executable='usv_gnc_node',
        name='usv_gnc_node',
        output='screen',
    )

    # ── Debug helper: echo EKF output when ekf_debug:=true ─────────────────
    ekf_echo = Node(
        package='topic_tools',
        executable='echo',
        name='ekf_echo',
        arguments=['/odometry/filtered'],
        output='screen',
        condition=IfCondition(ekf_debug),
    )

    return LaunchDescription([
        ekf_debug_arg,
        LogInfo(msg='[SEALEX] Starting navsat_transform_node ...'),
        navsat_node,
        LogInfo(msg='[SEALEX] Starting ekf_node ...'),
        ekf_node,
        LogInfo(msg='[SEALEX] Starting usv_gnc_node ...'),
        gnc_node,
        ekf_echo,
    ])
