
#include <ros/ros.h>

#include <stacked_stereo_transport/stereo_stacker.h>

class StereoStacker;

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_stacker");

  StereoStacker stacker;

  ros::spin();
  return 0;
}
