#ifndef CAMERA_CALIBRATION_IMAGE_PROC_H
#define CAMERA_CALIBRATION_IMAGE_PROC_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <string>
#include <memory>
#include <map>
#include <ros/console.h>
#include <math.h>
#include <vector>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

//For transform support

namespace camera_calibration_virtana{
  class ImageProcessor{

    public:

      ImageProcessor();

      ~ImageProcessor();

      void ImageCB(const sensor_msgs::ImageConstPtr& msg);

      void InitialiseSubscribers();

    private:
      ros::NodeHandle private_nh_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber image_sub_;
//      image_transport::Publisher image_pub_;
      std::string OPENCV_WINDOW;
  };
};

#endif