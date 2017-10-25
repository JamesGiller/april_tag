//
// adapted from ros example and april tag examples - palash
//
#include "april_tag_ros/april_tag_nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "AprilTags/Tag36h11.h"

namespace april_tag
{

void AprilTagNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle private_nh = getPrivateNodeHandle();

  april_tag::TagPublisher::Loggers loggers;
  loggers.log_info = [this](std::string what) -> void {NODELET_INFO(what.c_str());};
  loggers.log_warn = [this](std::string what) -> void {NODELET_WARN(what.c_str());};
  loggers.log_error = [this](std::string what) -> void {NODELET_ERROR(what.c_str());};

  try
  {
    tag_publisher_.reset(new TagPublisher(nh, private_nh, AprilTags::tagCodes36h11, std::move(loggers)));
  }
  catch(const std::runtime_error &e)
  {
    NODELET_ERROR(e.what());
  }
}
}

PLUGINLIB_EXPORT_CLASS(april_tag::AprilTagNodelet, nodelet::Nodelet)
