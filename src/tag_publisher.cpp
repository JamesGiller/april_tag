#include "april_tag/tag_publisher.h"

#include <algorithm>
#include <iterator>

namespace april_tag
{

TagPublisher::TagPublisher(ros::NodeHandle &nh, const AprilTags::TagCodes &tag_codes, DetectionContext base_context,
                           ErrorCallback on_error) :
  detection_context_{base_context},
  detect_tags_{tag_codes},
  it_{nh},
  on_error_{std::move(on_error)}
{
  tag_list_pub_ = nh.advertise<AprilTagList>("/april_tags", 100);
  image_sub_ = it_.subscribe("camera/image_raw", 1, &TagPublisher::detectAndPublishTags_, this);
}

void TagPublisher::detectAndPublishTags_(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    auto detections = detect_tags_(msg);

    if(detections.size() > 0)
    {
      detection_context_.time_and_place = msg->header;
      AprilTagList tag_list;
      std::transform(detections.begin(), detections.end(), std::back_inserter(tag_list.april_tags),
                     std::bind(tagDetectionToMsg, std::placeholders::_1, detection_context_));
      tag_list_pub_.publish(tag_list);
    }
  }
  catch (const std::runtime_error& e)
  {
    on_error_(std::string{e.what()});
  }
}
}
