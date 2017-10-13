//
// adapted from ros example and april tag examples - palash
//
#include <algorithm>
#include <functional>
#include <iterator>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include "april_tag/detect_tags_in_ros_image.h"
#include "april_tag/tag_detection_to_msg.h"
#include "april_tag/AprilTagList.h"
#include "AprilTags/Tag36h11.h"

class AprilTagNode
{
  // allow configurations for these:  
  double camera_focal_length_x_px_;
  double camera_focal_length_y_px_;
  double camera_principal_point_x_px_;
  double camera_principal_point_y_px_;
  double tag_size_m_;

  april_tag::DetectTagsInROSImage detect_tags_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher tag_list_pub_;
  
public:
  AprilTagNode(ros::NodeHandle &nh, const AprilTags::TagCodes &tag_codes) : 
    detect_tags_(tag_codes),
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
  }

  april_tag::AprilTag convertToMsg(const AprilTags::TagDetection &detection, const sensor_msgs::ImageConstPtr &in_img)
  {
    april_tag::DetectionContext detection_context = {in_img->header,
                                                     camera_focal_length_x_px_, camera_focal_length_y_px_,
                                                     camera_principal_point_x_px_, camera_principal_point_y_px_,
                                                     tag_size_m_};
    return april_tag::tagDetectionToMsg(detection, detection_context);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      auto detections = detect_tags_(msg);

      if(detections.size() > 0)
      {
        april_tag::AprilTagList tag_list;
        std::transform(detections.begin(), detections.end(), std::back_inserter(tag_list.april_tags),
                       std::bind(&AprilTagNode::convertToMsg, this, std::placeholders::_1, msg));
        tag_list_pub_.publish(tag_list);
      }
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR("%s", e.what());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "april_tag_node");
  ros::NodeHandle nh;
  AprilTagNode atn(nh, AprilTags::tagCodes36h11);
  ros::spin();
  return 0;
}
