RGBD Data Collection
====================

## Collect A Fixed Number of PCD Files from ROS Pointclouds

This will collect `N_CLOUDS` point clouds in the current directory from the given ROS
`PointCloud2` topic:

```bash
rosrun rgbd_data_collection pcd.bash N_CLOUDS /ros_topic_name
```
