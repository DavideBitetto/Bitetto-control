# Bitetto-control

roslaunch test_gazebo test.launch

roslaunch my_robot_urdf spawn_urdf.launch 

roslaunch test_control CT_controller.launch

rosrun my_controller topic_CT
