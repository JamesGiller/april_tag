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
    detection_context_.tag_size_m = tag_size_cm / 100.0;

    ros::NodeHandle private_node_handle("~");

    std::string camera_ns;
    if(!private_node_handle.getParam("camera_ns", camera_ns))
    {
      ROS_ERROR("Parameter 'camera_ns' is required but not set");
      exit(1);
    }

    private_node_handle.param<double>("focal_length_x_px", detection_context_.camera_focal_length_x_px, -1.);
    private_node_handle.param<double>("focal_length_y_px", detection_context_.camera_focal_length_y_px, -1.);
    private_node_handle.param<double>("principal_point_x_px", detection_context_.camera_principal_point_x_px, -1.);
    private_node_handle.param<double>("principal_point_y_px", detection_context_.camera_principal_point_y_px, -1.);

    if(detection_context_.camera_focal_length_x_px < 0. || detection_context_.camera_focal_length_y_px < 0.
       || detection_context_.camera_principal_point_x_px < 0. || detection_context_.camera_principal_point_y_px < 0.)
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
      detection_context_.camera_focal_length_x_px = camera_info->K[0];
      detection_context_.camera_focal_length_y_px = camera_info->K[4];
      detection_context_.camera_principal_point_x_px = camera_info->K[2];
      detection_context_.camera_principal_point_y_px = camera_info->K[5];
    }

    image_sub_ = it_.subscribe(camera_ns + "/image_raw", 1, &AprilTagNode::detectAndPublishTags, this);
    tag_list_pub_ = nh.advertise<april_tag::AprilTagList>("/april_tags", 100);
  }

  void detectAndPublishTags(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      auto detections = detect_tags_(msg);

      if(detections.size() > 0)
      {
        detection_context_.time_and_place = msg->header;
        april_tag::AprilTagList tag_list;
        std::transform(detections.begin(), detections.end(), std::back_inserter(tag_list.april_tags),
                       std::bind(april_tag::tagDetectionToMsg, std::placeholders::_1, detection_context_));
        tag_list_pub_.publish(tag_list);
      }
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR("%s", e.what());
    }
  }

private:
  april_tag::DetectionContext detection_context_;
  april_tag::DetectTagsInROSImage detect_tags_;
  image_transport::Subscriber image_sub_;
  image_transport::ImageTransport it_;
  ros::Publisher tag_list_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "april_tag_node");
  ros::NodeHandle nh;
  AprilTagNode atn(nh, AprilTags::tagCodes36h11);
  ros::spin();
  return 0;
}
