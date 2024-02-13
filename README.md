$ git clone -b <distro>-devel https://github.com/ros-industrial/universal_robot.git

$ git clone https://github.com/dairal/ur5-joint-position-control.git

$ sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers

$ git clone https://github.com/qleydon/RobotControl.git

$ catkin_make

$ source devel/setup.bash

$ roslaunch gazebo_ros empty_world.launch

$ rosrun gazebo_ros spawn_model -file `rospack find ur5-joint-position-control`/urdf/ur5_jnt_pos_ctrl.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5

new terminal

$ source devel/setup.bash

$ roslaunch ur5-joint-position-control ur5_joint_position_control.launch

new terminal
$ source devel/setup.bash

$ roslaunch ur5_control IK.launch
