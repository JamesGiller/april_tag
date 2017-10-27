//
// adapted from ros example and april tag examples - palash
//

#include <memory>

#include <ros/ros.h>

#include "april_tag_ros/tag_publisher.h"
#include "april_tag_ros/visualize_tags.h"
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
    std::unique_ptr<april_tag::VisualizeTags> visualize_tags;

    if(private_nh.param<bool>("publish_visualization_markers", false))
    {
      visualize_tags.reset(new april_tag::VisualizeTags(nh));
    }
    ros::spin();
    return 0;
  }
  catch(const std::runtime_error &e)
  {
    ROS_ERROR(e.what());
    return 1;
  }
}
