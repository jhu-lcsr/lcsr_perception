
#include <stacked_stereo_transport/stereo_stacker.h>

StereoStacker::StereoStacker () :
  nh_(),
  stereo_it_(nh_),
  sync_(16)
{
  // Initialize the info sequence indices to 0
  info_l_.header.seq = 0;
  info_r_.header.seq = 0;

  // Subscribe to input topics
  image_subf_l_.subscribe(stereo_it_, "left/image", 4);
  image_subf_r_.subscribe(stereo_it_, "right/image", 4);

  info_sub_l_ = nh_.subscribe("left/camera_info", 1, &StereoStacker::leftInfoCB, this);
  info_sub_r_ = nh_.subscribe("right/camera_info", 1, &StereoStacker::rightInfoCB, this);

  // Create stacked image publisher
  stacked_it_.reset(new image_transport::ImageTransport(ros::NodeHandle("stacked")));
  stacked_pub_ = stacked_it_->advertise("image", 1);

  sync_.connectInput(image_subf_l_, image_subf_r_);
  sync_.registerCallback(boost::bind(&StereoStacker::imageCB, this, _1, _2));
}

void StereoStacker::leftInfoCB(const sensor_msgs::CameraInfoConstPtr& info_l)
{
  info_l_ = *info_l;
}

void StereoStacker::rightInfoCB(const sensor_msgs::CameraInfoConstPtr& info_r)
{
  info_r_ = *info_r;
}

void StereoStacker::imageCB(
    const sensor_msgs::ImageConstPtr& image_l,
    const sensor_msgs::ImageConstPtr& image_r)
{
  // Don't do anything until we get the first camera info
  if(info_l_.header.seq == 0 and info_r_.header.seq == 0) {
    return;
  }

  cv_bridge::CvImageConstPtr cv_left = cv_bridge::toCvShare(image_l, "bgr8");
  cv_bridge::CvImageConstPtr cv_right = cv_bridge::toCvShare(image_r, "bgr8");

  const cv::Mat &left = cv_left->image;
  const cv::Mat &right = cv_right->image;

  if(left.type() != right.type()) { return; }
  if(left.cols != right.cols) { return; }

  // Create stacked image
  cv::Mat stacked(
      cv::Size(left.cols, left.rows + right.rows),
      left.type());
  cv_bridge::CvImage cv_stacked(
      image_l->header,
      "bgr8",
      stacked);

  left.copyTo(stacked(cv::Rect(0,0,left.cols,left.rows)));
  right.copyTo(stacked(cv::Rect(0,left.rows,left.cols,left.rows)));

  // Republish
  stacked_pub_.publish(cv_stacked.toImageMsg());
}
