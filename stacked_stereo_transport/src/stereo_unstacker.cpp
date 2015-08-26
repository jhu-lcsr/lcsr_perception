

#include <stacked_stereo_transport/stereo_unstacker.h>

StereoUnstacker::StereoUnstacker() :
  nh_(),
  stacked_it_(ros::NodeHandle("stacked")),
  left_it_(ros::NodeHandle("unstacked/left")),
  right_it_(ros::NodeHandle("unstacked/right")),
  info_l_(boost::make_shared<sensor_msgs::CameraInfo>()),
  info_r_(boost::make_shared<sensor_msgs::CameraInfo>()),
  left_img_msg_(boost::make_shared<sensor_msgs::Image>()),
  right_img_msg_(boost::make_shared<sensor_msgs::Image>()),
  pub_rate_(0.0)
{
  // Initialize the info sequence indices to 0
  info_l_->header.seq = 0;
  info_r_->header.seq = 0;

  // Subscribe to input topics
  ros::TransportHints ros_hints;
  ros_hints.tcpNoDelay();
  image_transport::TransportHints hints("compressed", ros_hints);
  stacked_sub_ = stacked_it_.subscribe("image", 2, &StereoUnstacker::imageCB, this, hints);

  info_sub_l_ = nh_.subscribe("left/camera_info", 1, &StereoUnstacker::leftInfoCB, this);
  info_sub_r_ = nh_.subscribe("right/camera_info", 1, &StereoUnstacker::rightInfoCB, this);

  // Create stacked image publisher
  cam_pub_l_ = left_it_.advertiseCamera("image", 4);
  cam_pub_r_ = right_it_.advertiseCamera("image", 4);
}

void StereoUnstacker::leftInfoCB(const sensor_msgs::CameraInfoConstPtr& info_l)
{
  // Cache left camera info
  *info_l_ = *info_l;
}

void StereoUnstacker::rightInfoCB(const sensor_msgs::CameraInfoConstPtr& info_r)
{
  // Cache left camera info
  *info_r_ = *info_r;
}

void StereoUnstacker::imageCB(
    const sensor_msgs::ImageConstPtr& image_stacked)
{
  ros::WallTime start = ros::WallTime::now();
  // Don't do anything until we get the first camera info
  if(info_l_->header.seq == 0 and info_r_->header.seq == 0) {
    ROS_WARN_STREAM("Still waiting for camera info...");
    return;
  }

  // Copy the headers to republish the cached camera info
  info_l_->header = image_stacked->header;
  info_r_->header = image_stacked->header;

  cv_bridge::CvImageConstPtr cv_stacked = cv_bridge::toCvShare(image_stacked, "bgr8");
  const cv::Mat &stacked = cv_stacked->image;

  // Unstack images
  cv_bridge::CvImage cv_left(
      image_stacked->header,
      "bgr8",
      stacked(cv::Rect(0,0,stacked.cols,stacked.rows/2)));
  cv_bridge::CvImage cv_right(
      image_stacked->header,
      "bgr8",
      stacked(cv::Rect(0,stacked.rows/2,stacked.cols,stacked.rows/2)));

  // Call user callback
  ros::Time stamp = image_stacked->header.stamp;
  if(user_cb_) {
    stamp = user_cb_(image_stacked->header, left_img_msg_, right_img_msg_);
  }

  if(stamp != image_stacked->header.stamp) {
    ROS_DEBUG_STREAM("Rewrote stacked stereo stamp! "<<image_stacked->header.stamp<<" -> "<<stamp);
  }

  // Republish
  cv_left.toImageMsg(*left_img_msg_);
  info_l_->header.stamp = stamp;
  left_img_msg_->header.stamp = stamp;
  cam_pub_l_.publish(left_img_msg_, info_l_);

  cv_right.toImageMsg(*right_img_msg_);
  info_r_->header.stamp = stamp;
  right_img_msg_->header.stamp = stamp;
  cam_pub_r_.publish(right_img_msg_, info_r_);

  // Measure publication rate
  ros::WallTime pub_time = ros::WallTime::now();
  pub_rate_ = 0.1*1.0/(pub_time - last_pub_time_).toSec() + 0.9*pub_rate_;
  last_pub_time_ = pub_time;
  ROS_DEBUG_STREAM(pub_rate_<<" Hz: Pub in "<<(pub_time - start).toSec()<<" s");
}

void StereoUnstacker::registerCB(const StereoCallbackType &user_cb) {
  user_cb_ = user_cb;
}
