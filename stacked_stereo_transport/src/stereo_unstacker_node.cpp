

#include <stacked_stereo_transport/stereo_unstacker.h>

#include <ros/ros.h>

class StereoUnstacker;

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_unstacker");

  StereoUnstacker unstacker;

  ros::spin();
  return 0;
}
