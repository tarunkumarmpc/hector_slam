# hector_slam


clone the repository 

mkdir -p ~/catkin_ws/src

catkin_init_workspace

cd ~/catkin_ws


After completion of catkin make 

(i)cd catkin_ws

. devel/setup.bash

Roscore



(ii)new terminal 

cd catkin_ws

. devel/setup.bash

roslaunch ros0xrobot ros0xrobot_minimal.launch


(iii)New terminal

cd catkin_ws

. devel/setup.bash

roslaunch rplidar_ros rplidar.launch

(iv)New terminal on working pc 

cd catkin_ws

. devel/setup.bash

roslaunch ros0xrobot ros0xrobot_teleop.launch


(v)New terminal

cd catkin_ws

. devel/setup.bash

roslaunch hector_slam_launch tutorial.launch


(vi)Can directly save in rviz 
or 

rosrun map_server map_saver -f /home/Desktop/maps/map0
