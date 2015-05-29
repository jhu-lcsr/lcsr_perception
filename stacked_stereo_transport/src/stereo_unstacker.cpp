

#include <stacked_stereo_transport/stereo_unstacker.h>

StereoUnstacker::StereoUnstacker() :
  nh_(),
  stacked_it_(ros::NodeHandle("stacked")),
  unstacked_it_(ros::NodeHandle("unstacked"))
{
  // Initialize the info sequence indices to 0
  info_l_.header.seq = 0;
  info_r_.header.seq = 0;

  // Subscribe to input topics
  stacked_sub_ = stacked_it_.subscribe("image", 4, &StereoUnstacker::imageCB, this);

  info_sub_l_ = nh_.subscribe("left/camera_info", 1, &StereoUnstacker::leftInfoCB, this);
  info_sub_r_ = nh_.subscribe("right/camera_info", 1, &StereoUnstacker::rightInfoCB, this);

  // Create stacked image publisher
  cam_pub_l_ = unstacked_it_.advertiseCamera("left", 1);
  cam_pub_r_ = unstacked_it_.advertiseCamera("right", 1);
}

void StereoUnstacker::leftInfoCB(const sensor_msgs::CameraInfoConstPtr& info_l)
{
  // Cache left camera info
  info_l_ = *info_l;
}

void StereoUnstacker::rightInfoCB(const sensor_msgs::CameraInfoConstPtr& info_r)
{
  // Cache left camera info
  info_r_ = *info_r;
}

void StereoUnstacker::imageCB(
    const sensor_msgs::ImageConstPtr& image_stacked)
{
  // Don't do anything until we get the first camera info
  if(info_l_.header.seq == 0 and info_r_.header.seq == 0) {
    return;
  }

  // Copy the headers to republish the cached camera info
  info_l_.header = image_stacked->header;
  info_r_.header = image_stacked->header;

  cv_bridge::CvImageConstPtr cv_stacked = cv_bridge::toCvShare(image_stacked, "bgr8");
  const cv::Mat &stacked = cv_stacked->image;

  // Unstack images
  cv::Mat left(stacked, cv::Rect(0,0,stacked.cols,stacked.rows/2));
  cv::Mat right(stacked, cv::Rect(0,stacked.rows/2,stacked.cols,stacked.rows/2));

  cv_bridge::CvImage cv_left(
      image_stacked->header,
      "bgr8",
      left);
  cv_bridge::CvImage cv_right(
      image_stacked->header,
      "bgr8",
      right);

  // Republish
  cam_pub_l_.publish(cv_left.toImageMsg(), sensor_msgs::CameraInfoConstPtr(new sensor_msgs::CameraInfo(info_l_)));
  cam_pub_r_.publish(cv_right.toImageMsg(), sensor_msgs::CameraInfoConstPtr(new sensor_msgs::CameraInfo(info_r_)));

  // Call user callback
  if(user_cb_) {
    user_cb_(cv_left.toImageMsg(), cv_right.toImageMsg());
  }
}

void StereoUnstacker::registerCB(const StereoCallbackType &user_cb) {
  user_cb_ = user_cb;
}
