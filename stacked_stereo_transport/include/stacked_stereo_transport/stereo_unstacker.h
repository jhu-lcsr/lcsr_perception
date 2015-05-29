#ifndef __STACKED_STEREO_TRANSPORT_STEREO_UNSTACKER_H
#define __STACKED_STEREO_TRANSPORT_STEREO_UNSTACKER_H

#include <ros/ros.h>
#include <ros/names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>

class StereoUnstacker {
public:

  typedef boost::function< void(const sensor_msgs::ImageConstPtr &, const sensor_msgs::ImageConstPtr &)> StereoCallbackType;

  StereoUnstacker();

  void imageCB(const sensor_msgs::ImageConstPtr& stacked_image);
  void leftInfoCB(const sensor_msgs::CameraInfoConstPtr& info_l);
  void rightInfoCB(const sensor_msgs::CameraInfoConstPtr& info_r);

  void registerCB(const StereoCallbackType &user_cb);

private:
  ros::NodeHandle nh_;

  // Cached camera info
  sensor_msgs::CameraInfo info_l_;
  sensor_msgs::CameraInfo info_r_;

  // Input
  image_transport::ImageTransport stacked_it_;
  image_transport::Subscriber stacked_sub_;
  ros::Subscriber info_sub_l_, info_sub_r_;

  // Output
  image_transport::ImageTransport left_it_, right_it_;
  image_transport::CameraPublisher cam_pub_l_, cam_pub_r_;

  // User callback
  StereoCallbackType user_cb_;
};

#endif // ifndef __STACKED_STEREO_TRANSPORT_STEREO_STACKER_H

