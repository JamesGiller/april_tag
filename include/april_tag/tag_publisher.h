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
class TagPublisher
{
public:
  using ErrorCallback = std::function<void(std::string what)>;
  /**
   * @brief this will subscribe to 'camera/image_raw' topic and publish detected tags to '/april_tags'
   *
   *        You should remap these topic names.
   * @param nh
   * @param tag_codes family of tags that should be detected
   * @param base_context all members should be initialized except DetectionContext::time_and_place
   */
  TagPublisher(ros::NodeHandle &nh, const AprilTags::TagCodes &tag_codes, DetectionContext base_context,
               ErrorCallback on_error);

private:
  void detectAndPublishTags_(const sensor_msgs::ImageConstPtr &msg);

  DetectionContext detection_context_;
  DetectTagsInROSImage detect_tags_;
  image_transport::ImageTransport it_;
  ros::Publisher tag_list_pub_;
  image_transport::Subscriber image_sub_;
  ErrorCallback on_error_;
};
}

#endif // APRIL_TAG_TAG_PUBLISHER_H
