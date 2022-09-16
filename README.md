# construct_solo12

mkdir -p construct_solo_ws/src

cd construct_solo_ws/src

git clone https://github.com/sgmurray/construct_solo12.git .

cd construct_solo_ws

source /opt/ros/foxy/setup.bash

colcon build

source install/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE=~/Ricardo_ws/src/ros2_control_solo12/ros2_description_solo12/config/FastRTPS.xml

sudo ls; ros2 launch ros2_control_bolt_bringup bolt_system_position_only.launch.py

To test the controller I have selected 4 of the least problematic motors:

ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.0
- 0.674
- -1.348
- 0.0
- 0.0
- 0.0
- 0.0
- 0.674
- -1.348
- 0.0
- 0.0
- 0.0"
