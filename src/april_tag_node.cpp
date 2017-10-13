//
// adapted from ros example and april tag examples - palash
//
#include <algorithm>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "april_tag/tag_detection_to_msg.h"
#include "april_tag/AprilTag.h" // rosmsg
#include "april_tag/AprilTagList.h" // rosmsg
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

class AprilTagNode
{
  static const std::string OPENCV_WINDOW;

  // allow configurations for these:  
  bool  show_debug_image_;
  sensor_msgs::ImageConstPtr latest_image_;
  double camera_focal_length_x_px_;
  double camera_focal_length_y_px_;
  double camera_principal_point_x_px_;
  double camera_principal_point_y_px_;
  double tag_size_m_;

  AprilTags::TagDetector tag_detector_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher tag_list_pub_;
  
public:
  AprilTagNode(ros::NodeHandle &nh, const AprilTags::TagCodes &tag_codes) : 
    tag_detector_(tag_codes),
    it_(nh) 
  {
    double tag_size_cm;
    if(!nh.getParam("tag_size_cm", tag_size_cm))
    {
      ROS_ERROR("Parameter 'tag_size_cm' is required but not set");
      exit(1);
    }
    tag_size_m_ = tag_size_cm / 100.0;

    ros::NodeHandle private_node_handle("~");

    std::string camera_ns;
    if(!private_node_handle.getParam("camera_ns", camera_ns))
    {
      ROS_ERROR("Parameter 'camera_ns' is required but not set");
      exit(1);
    }

    private_node_handle.param<double>("focal_length_x_px", camera_focal_length_x_px_, -1.);
    private_node_handle.param<double>("focal_length_y_px", camera_focal_length_y_px_, -1.);
    private_node_handle.param<double>("principal_point_x_px", camera_principal_point_x_px_, -1.);
    private_node_handle.param<double>("principal_point_y_px", camera_principal_point_y_px_, -1.);

    if(camera_focal_length_x_px_ < 0. || camera_focal_length_y_px_ < 0.
       || camera_principal_point_x_px_ < 0. || camera_principal_point_y_px_ < 0.)
    {
      ROS_INFO("Camera intrinsics not supplied. Waiting to fetch from topic '%s/camera_info'", camera_ns.c_str());
      auto camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_ns + "/camera_info", nh);
      if(!camera_info)
      {
        ROS_INFO("Interrupted by node shutdown.");
        exit(0);
      }
      if(std::all_of(camera_info->K.begin(), camera_info->K.end(), [](double k) { return k == 0;}))
      {
        ROS_WARN("Intrinsics of CameraInfo fetched from topic '%s/camera_info' are all zero."
                 " Make sure your camera publishes the correct intrinsics", camera_ns.c_str());
      }
      /* sensor_msgs/CameraInfo only describes contents of intrinsic camera matrix in comments in msg file, so
         there is no point having named constants here, sorry. */
      camera_focal_length_x_px_ = camera_info->K[0];
      camera_focal_length_y_px_ = camera_info->K[4];
      camera_principal_point_x_px_ = camera_info->K[2];
      camera_principal_point_y_px_ = camera_info->K[5];
    }

    ROS_INFO_STREAM("got focal length " << camera_focal_length_x_px_ << ", " << camera_focal_length_y_px_);
    ROS_INFO_STREAM("got principal point " << camera_principal_point_x_px_ << ", " << camera_principal_point_y_px_);

    image_sub_ = it_.subscribe(camera_ns + "/image_raw", 1, &AprilTagNode::imageCb, this);
    tag_list_pub_ = nh.advertise<april_tag::AprilTagList>("/april_tags", 100);

    // be run simultaneously while using different parameters.
    private_node_handle.param<bool>("show_debug_image", show_debug_image_, false);

    if (show_debug_image_)
    {
      cv::namedWindow(OPENCV_WINDOW);
    }
  }

  ~AprilTagNode()
  {
    if (show_debug_image_)
    {
     cv::destroyWindow(OPENCV_WINDOW);
    }
  }

  april_tag::AprilTag convert_to_msg(AprilTags::TagDetection &detection) {
    april_tag::DetectionContext detection_context = {latest_image_->header,
                                                     camera_focal_length_x_px_, camera_focal_length_y_px_,
                                                     camera_principal_point_x_px_, camera_principal_point_y_px_,
                                                     tag_size_m_};
    return april_tag::tagDetectionToMsg(detection, detection_context);
  }

  void processCvImage(cv_bridge::CvImagePtr cv_ptr) 
  {
    cv::Mat image_gray;
    cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
    vector<AprilTags::TagDetection> detections = tag_detector_.extractTags(image_gray);
    vector<april_tag::AprilTag> tag_msgs;

    for (int i=0; i < detections.size(); ++i)
    {
      if (show_debug_image_)
      {
        detections[i].draw(cv_ptr->image);
      }
      tag_msgs.push_back(convert_to_msg(detections[i]));
    }

    if(detections.size() > 0)
    { // take this out if you want absence notificaiton
      april_tag::AprilTagList tag_list;
      tag_list.april_tags = tag_msgs;
      tag_list_pub_.publish(tag_list);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    latest_image_ = msg;
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

    processCvImage(cv_ptr);

    if (show_debug_image_)
    {
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
    }
  }
};

const std::string AprilTagNode::OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "april_tag_node");
  ros::NodeHandle nh;
  AprilTagNode atn(nh, AprilTags::tagCodes36h11);
  ros::spin();
  return 0;
}
