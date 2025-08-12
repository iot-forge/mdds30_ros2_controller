ros2 topic pub -1 /mdds30_controller/motor_right std_msgs/msg/Int16 '{data: 50}'

ros2 run teleop_twist_joy teleop_node --ros-args --params-file ~/ros2_ws/src/mdds30_ros2_controller-main/config/teleop.yaml

ros2 topic echo /joy

ros2 run joy joy_node

ros2 launch mdds30_ros2_controller mdds30_controller.launch.py


 source ros2_ws/src/mdds30_ros2_controller-main/install/setup.bash 

 sudo usermod -a -G dialout $USER
 
 sudo apt install ros-kilted-teleop-twist-joy joystick
 
 sudo apt install ros-kilted-joy


 colcon build --symlink-install
 
 source install/setup.bash



