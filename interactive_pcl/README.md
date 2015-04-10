Interactive PCL
===============

Control PCL filters with interactive markers.

First, launch a few filters / markers:

```
roslaunch pass_though.launch __ns:=/camera
roslaunch pass_though.launch in:=1 out:=2 __ns:=/camera
roslaunch pass_though.launch in:=2 out:=3 __ns:=/camera
```

Next, add interactive markers, planes, and point clouds to RViz.

