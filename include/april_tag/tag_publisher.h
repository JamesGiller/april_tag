/** \author James Giller */
#ifndef APRIL_TAG_TAG_PUBLISHER_H
#define APRIL_TAG_TAG_PUBLISHER_H

#include <functional>

#include <image_transport/image_transport.h>
#include <ros/node_handle.h>

#include "april_tag/detect_tags_in_ros_image.h"
#include "april_tag/tag_detection_to_msg.h"
#include "april_tag/AprilTagList.h"

namespace april_tag
{
/**
 * @brief Subscribes to a sensor_msgs/Image topic and publishes information when tags are detected in images
 */
class TagPublisher
{
public:
  using LoggingCallback = std::function<void(std::string what)>;

  /**
   * @brief Aggregates functions to log messages at different levels
   */
  struct Loggers
  {
    LoggingCallback log_info;
    LoggingCallback log_warn;
    LoggingCallback log_error;
  };

  /**
   * @brief this will subscribe to 'camera/image_raw' topic and publish detected tags to '/april_tags'
   *
   *        You should remap these topic names if necessary.
   * @param nh
   * @param private_nh
   * @param tag_codes family of tags that should be detected
   * @param loggers logging functions to use
   */
  TagPublisher(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const AprilTags::TagCodes &tag_codes, Loggers loggers);

private:
  void detectAndPublishTags_(const sensor_msgs::ImageConstPtr &msg);

  DetectionContext detection_context_;
  DetectTagsInROSImage detect_tags_;
  image_transport::ImageTransport it_;
  ros::Publisher tag_list_pub_;
  image_transport::Subscriber image_sub_;
  Loggers loggers_;
};
}

#endif // APRIL_TAG_TAG_PUBLISHER_H
