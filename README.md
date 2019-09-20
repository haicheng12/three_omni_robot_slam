# three_omni_robot_slam
The three omni robot slam in the real environment.

1.Required dependent environment:
----------------
- ubuntu 16.04
- ros kinetic

```
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-openslam-gmapping
sudo apt-get install ros-kinetic-slam-gmapping
```
If you use hokuyo laser,you should install this driven:
```
sudo apt-get install ros-kinetic-driver-base
```

2.Install cartorgrapher:
-----------------
reference:http://www.cnblogs.com/wenhust/p/6047258.html

3.Get serial port permission forever:
-----------------

reference:https://blog.csdn.net/wilylcyu/article/details/52051964

and you can bind serial port end number,
reference:https://blog.csdn.net/xiangbin099/article/details/79676053
and use this order to let it take effect:
```
sudo udevadm trigger

ls -l /dev/
```


4.Gmapping and navigation
------------------

cartorgrapher slam:
```
roslaunch test_slam cartorgrapher.launch
```
And you can use the keyboard "W S A D Q E" to control the robot to collect map information.Then use this command to save the map:
```
rosrun map_sever map_saver -f map
```
Next,you can use these commands to let the robot navigate automatically:
```
roslaunch test_slam navigation.launch
roslaunch test_slam navigation_rviz.launch
```

5.Let the .sh file start when pc start:
-----------------
sudo gedit start.sh
```
#! /bin/bash  
source /opt/ros/kinetic/setup.sh 
source /home/cheng/catkin_ws/devel/setup.bash 
roslaunch xx xx.launch &
sleep 2 
roslaunch xxx xxx.launch &
 
exit 0

```
and then add:
gnome-terminal -x  /home/cheng/start.sh

to the starting up.
