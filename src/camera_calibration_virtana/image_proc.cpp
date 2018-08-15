#include "camera_calibration_virtana/image_proc.h"

namespace camera_calibration_virtana{

  ImageProcessor::ImageProcessor():private_nh_("~"), it_(ros::NodeHandle())
  {
    OPENCV_WINDOW = "Image Window";
    InitialiseSubscribers();
  };

  ImageProcessor::~ImageProcessor()
  {

  };

  void ImageProcessor::InitialiseSubscribers()
  {
    image_sub_ = it_.subscribe("/camera1/image_raw", 1, &ImageProcessor::ImageCB, this);
  };

  void ImageProcessor::ImageCB(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<cv::Point2f> corners;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    bool patternfound = cv::findChessboardCorners(cv_ptr->image, cvSize(4,4), corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK);

    if(patternfound)
    {
      ROS_INFO("Found board");
      ROS_INFO("Number of corners found: , %lu", corners.size());
      cv::drawChessboardCorners(cv_ptr->image, cvSize(4,4), cv::Mat(corners), patternfound);
    }

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  };

}