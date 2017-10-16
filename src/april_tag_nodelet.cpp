//
// adapted from ros example and april tag examples - palash
//
#include "april_tag/april_tag_nodelet.h"

#include <algorithm>
#include <functional>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "AprilTags/Tag36h11.h"

namespace april_tag
{

void AprilTagNodelet::onInit()
{
  initialization_thread_ = getNodeHandle().createTimer(ros::Duration(0.001),
                                                       [this](const ros::TimerEvent &timer_event) -> void
  {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle private_nh = getPrivateNodeHandle();

    april_tag::DetectionContext detection_context;

    double tag_size_cm;
    if(!nh.getParam("tag_size_cm", tag_size_cm))
    {
      NODELET_ERROR("Parameter 'tag_size_cm' is required but not set");
      return;
    }
    detection_context.tag_size_m = tag_size_cm / 100.0;


    std::string camera_ns;
    if(!private_nh.getParam("camera_ns", camera_ns))
    {
      NODELET_ERROR("Parameter 'camera_ns' is required but not set");
      return;
    }

    double fx, fy, cx, cy;
    private_nh.param<double>("focal_length_x_px", fx, -1.);
    private_nh.param<double>("focal_length_y_px", fy, -1.);
    private_nh.param<double>("principal_point_x_px", cx, -1.);
    private_nh.param<double>("principal_point_y_px", cy, -1.);

    if(fx < 0. || fy < 0. || cx < 0. || cy < 0.)
    {
      NODELET_INFO("Camera intrinsics not supplied. Waiting to fetch from topic '%s/camera_info'", camera_ns.c_str());

      auto camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_ns + "/camera_info", nh);
      if(!camera_info)
      {
        NODELET_INFO("Interrupted by node shutdown.");
        return;
      }
      if(std::all_of(camera_info->K.begin(), camera_info->K.end(), [](double k) { return k == 0;}))
      {
        NODELET_WARN("Intrinsics of CameraInfo fetched from topic '%s/camera_info' are all zero."
                 " Make sure your camera publishes the correct intrinsics", camera_ns.c_str());
      }
      detection_context.setIntrinsicsFromCameraInfo(*camera_info);
    }
    else
    {
      detection_context.setCameraIntrinsics(fx, fy, cx, cy);
    }

    tag_publisher_.reset(new TagPublisher(nh, AprilTags::tagCodes36h11, std::move(detection_context),
                                          [this](std::string what) -> void
    {
      NODELET_ERROR("Error: %s", what.c_str());
    }));
  }, true);
}
}

PLUGINLIB_EXPORT_CLASS(april_tag::AprilTagNodelet, nodelet::Nodelet)
