## mmwave
# sensor open
```
$ cd catkin_ws && catkin_make
$ source devel/setup.sh
```
You can choose following methods to launch mmWave sensor. 
If you got error when conduct roslaunch, press RESET button on your AWR1443Boost board, wait about 5 seconds then re-try.
```
$ roslaunch ti_mmwave_rospkg rviz_1443_3d.launch config:=3d_mid_range
$ roslaunch ti_mmwave_rospkg rviz_1443_3d.launch config:=3d_short_range
$ roslaunch ti_mmwave_rospkg rviz_1443_3d.launch config:=3d
```

# mmwave_detection
rosrun mmwave_detection simple_tracking_node
