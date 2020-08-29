# stack_brick
The project was finished on 06/2019.  
  
  
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

The final demo could be found in [media](media)  
