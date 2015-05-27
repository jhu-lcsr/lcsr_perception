```bash
roslaunch gazebo.launch gui:=true
roslaunch kinect.launch
roslaunch drill.launch
roslaunch objrec_ransac.launch input_topic:=/camera/depth_registered/points
```

Then run rviz.

```
rosrun rviz rviz -d view.rviz
```
