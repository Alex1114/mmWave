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
$ sudo chmod 777 /dev/ttyUSB*
```
```
$ roslaunch ti_mmwave_rospkg multi_6843_0.launch
$ roslaunch ti_mmwave_rospkg multi_6843_1.launch
$ roslaunch ti_mmwave_rospkg multi_6843_2.launch
$ roslaunch ti_mmwave_rospkg multi_6843_3.launch
```
- If you want to open 4 mmwave, you also can run command
```
$ roslaunch ti_mmwave_rospkg mmwave_6843_all.launch

```
## base (conbine four topic)
```
$ roslaunch ti_mmwave_rospkg base.launch

```
## mmwave_detection (filter)
```
rosrun mmwave_detection simple_tracking_node
```



