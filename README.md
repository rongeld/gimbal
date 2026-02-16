1. Run Gazebo with the UAV model that has the gimbal_kinematic_control
     plugin.
     That plugin listens on ROS2 topic /set_joint_trajectory and applies yaw/
     pitch/roll directly to simulated gimbal joints.
  2. Stream the Gazebo camera to local UDP with:
  3. Mirror real SIYI attitude into Gazebo with:
     SIYI_SDK_PATH=/home/rongeld/Documents/Projects/siyi
     python /scripts/siyi_attitude_mirror.py --ip {IP}
     --poll-hz 120 --publish-hz 120 --invert-pitch
  5. That script uses siyi_sdk.py to poll real gimbal attitude over UDP,
     converts degrees to radians, and publishes JointTrajectory to /
     set_joint_trajectory at 120 Hz.
