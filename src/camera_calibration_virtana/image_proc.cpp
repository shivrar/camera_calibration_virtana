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

  void ImageProcessor::SetCamInfo(const sensor_msgs::CameraInfoConstPtr& info)
  {
    camera_info_ = *info;
  };

  void ImageProcessor::InitialiseSubscribers()
  {
    //TODO: Maybe a param here.
    cam_info_sub_ = ros::NodeHandle().subscribe("/camera1/camera_info", 1, &ImageProcessor::SetCamInfo, this);
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
      for(std::vector<cv::Point2f>::iterator corners_iter = corners.begin(); corners_iter != corners.end(); corners_iter++)
      {

         ROS_INFO("Points on Image <x,y>: <%f,%f> \n", corners_iter->x, corners_iter->y);


      }

      cv::drawChessboardCorners(cv_ptr->image, cvSize(4,4), cv::Mat(corners), patternfound);

    }

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  };

}