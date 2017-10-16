//
// adapted from ros example and april tag examples - palash
//

#include <ros/ros.h>

#include "april_tag/tag_publisher.h"
#include "AprilTags/Tag36h11.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "april_tag_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  april_tag::TagPublisher::Loggers loggers;
  loggers.log_info = [](std::string what) -> void {ROS_INFO(what.c_str());};
  loggers.log_warn = [](std::string what) -> void {ROS_WARN(what.c_str());};
  loggers.log_error = [](std::string what) -> void {ROS_ERROR(what.c_str());};

  try
  {
    april_tag::TagPublisher tag_publisher{nh, private_nh, AprilTags::tagCodes36h11, std::move(loggers)};
    ros::spin();
    return 0;
  }
  catch(const std::runtime_error &e)
  {
    ROS_ERROR(e.what());
    return 1;
  }
}
