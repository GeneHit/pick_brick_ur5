# simulation of stack-brick system  
The project was finished on 06/2019.  
The project demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a Kinect camera to detect the bricks (brick_det_2base_4.py). The position and orientation is published. UR5(brick_stackV4_6.py) searches for brick and picks it(vacuumpump_srv.py). After UR5 picks a brick, controller will take the UR5 motion to place to place the brick in sorted.  
The final demo could be found in [demo](https://youtu.be/KV2r1aGEdeI)
  
# Neccesary Resources  
## 1.[universal_robot](https://github.com/ros-industrial/universal_robot)  
for UR5.  
## 2.MoveIt!  
[official installation tuturial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)  
## 3.[gazebo_ros_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher.git)  
for vacuum gripper.  
## 4.OpenCV (at least 2.7 version)  
## 5.ikfast kinematic solver
a installation toturial [ikfast](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html)  
  
  
# run command
open four teminals:  
first ter
>roslaunch pick_brick_ur5 initialize_bricks.launch  

second ter  
>rosrun pick_brick_ur5 vacuumpump_srv.py  

third ter  
>rosrun pick_brick_ur5 brick_det_2base_4.py  

fourth ter
>rosrun pick_brick_ur5 brick_stackV4_6.py  
  
