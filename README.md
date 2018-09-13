# three_omni_robot_slam
The three omni robot slam in the real environment.

The picture of robot:
![The picture of robot](https://img-blog.csdn.net/20180909165143463?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L21lbmd4aWFuZzI0MjU=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

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
