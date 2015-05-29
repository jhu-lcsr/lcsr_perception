#ifndef __STACKED_STEREO_TRANSPORT_STEREO_STACKER_H
#define __STACKED_STEREO_TRANSPORT_STEREO_STACKER_H

#include <ros/ros.h>
#include <ros/names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

class StereoStacker {
private:
  ros::NodeHandle nh_;

  // Cached camera info
  sensor_msgs::CameraInfo info_l_;
  sensor_msgs::CameraInfo info_r_;

  // Input
  image_transport::ImageTransport stereo_it_;

  image_transport::SubscriberFilter image_subf_l_, image_subf_r_;
  ros::Subscriber info_sub_l_, info_sub_r_;

  // Synchronizer
  message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image> > sync_;

  // Output
  boost::shared_ptr<image_transport::ImageTransport> stacked_it_;
  image_transport::Publisher stacked_pub_;

public:
  StereoStacker();

  void leftInfoCB(const sensor_msgs::CameraInfoConstPtr& info_l);

  void rightInfoCB(const sensor_msgs::CameraInfoConstPtr& info_r);

  void imageCB(const sensor_msgs::ImageConstPtr& image_l,
               const sensor_msgs::ImageConstPtr& image_r);
};

#endif // ifndef __STACKED_STEREO_TRANSPORT_STEREO_STACKER_H
