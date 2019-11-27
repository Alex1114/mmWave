# mmwave
## ti_mmwave_rospkg (sensor open)
```
$ cd catkin_ws && catkin_make
$ source devel/setup.sh
```
- You can choose following methods to launch mmWave sensor. 
- If you got error when conduct roslaunch, press RESET button on your board.
- You have to check the port in .launch file is correct. 
```
$ cd /dev
$ ls
```
```
$ roslaunch ti_mmwave_rospkg multi_6843_0.launch
$ roslaunch ti_mmwave_rospkg multi_6843_1.launch
$ roslaunch ti_mmwave_rospkg multi_6843_2.launch
$ roslaunch ti_mmwave_rospkg multi_6843_3.launch
```

## mmwave_detection (filter)
```
rosrun mmwave_detection simple_tracking_node
```

## pcl_perception (conbine four topic)
```
$ roslaunch pcl_perception base.launch
```

