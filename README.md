# inverse kinematics

to run this cd devel and $ source setup.zsh or setup.sh  
then to launch the visualizer: $ roslaunch arm_moveit_config demo.launch  
then to launch the listener: $ rosrun arm_moveit_config arm_controller.py  
then to launch the test publisher: $ rosrun arm_moveit_config test_pub.py

in rvis the arm should start moving. 

To actually get it to run for real demo.launch will need to be update it so that we aren't talking to a fake controller
http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/controller_configuration/controller_configuration_tutorial.html
http://wiki.ros.org/ros_control