#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>

static const std::string ORIGINAL_WINDOW = "Original Image";
static const std::string ARUCO_MARKER_WINDOW = "Find Marker";
cv::Mat cameraP = (cv::Mat_<float>(3,3) << 970.63427734375, 0.0, 1022.773681640625, 0.0, 970.6431884765625, 781.4906005859375, 0.0, 0.0, 1.0);
cv::Mat dist = (cv::Mat_<float>(1,5) << 0.5164358615875244, -2.606694221496582, 0.00045736812171526253, -0.00019684531434904784, 1.499117374420166);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/rgb/image_raw", 1, &ImageConverter::imageCb, this);
  //  cv::namedWindow(ORIGINAL_WINDOW);
    cv::namedWindow(ARUCO_MARKER_WINDOW);
  }

  ~ImageConverter()
  {
  //  cv::destroyWindow(ORIGINAL_WINDOW);
    cv::destroyWindow(ARUCO_MARKER_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);


    cv::Mat findcode_raw;
    cv_ptr->image.copyTo(findcode_raw);

    //resize the pic window
    cv::Mat findcode_show;
    cv::Size size(640,480);
    cv::resize(findcode_raw,findcode_show,size);                    //resize image

    std::vector<int> ids;
    std::vector< std::vector<cv::Point2f> > corners;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    cv::aruco::detectMarkers(findcode_show, dictionary, corners, ids);

    if (ids.size() > 0)
    {
      //draw Markers
      std::cout << "Number of Markers we detected: " << ids.size() << std::endl;
      cv::aruco::drawDetectedMarkers(findcode_show, corners, ids);
      //detect Pose
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraP, dist, rvecs, tvecs);
      //draw axis for each marker
      for(int i=0; i<ids.size(); i++)
      {
          cv::aruco::drawAxis(findcode_show, cameraP, dist, rvecs[i], tvecs[i], 0.1);
          std::cout << "RotMat of Arucomarker " << i+1 <<" is " << rvecs[i]<< std::endl;
          std::cout << "TransMat of Arucomarker " << i+1 <<" is "<< tvecs[i]<< std::endl;
      }
    }

    //show the picture with marker
    cv::imshow(ARUCO_MARKER_WINDOW, findcode_show);
    //End here
    cv::waitKey(3);


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "findmarker");
  ImageConverter ic;
  ros::spin();
  return 0;
}

