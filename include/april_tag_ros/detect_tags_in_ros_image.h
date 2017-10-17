/** \author James Giller */

#ifndef APRIL_TAG_DETECT_IN_ROS_IMAGE_H
#define APRIL_TAG_DETECT_IN_ROS_IMAGE_H

#include <sensor_msgs/Image.h>

#include "AprilTags/TagDetector.h"

namespace april_tag
{
/**
 * @brief Adapter for AprilTags library that accepts ROS images (sensor_msgs/Image) as input
 */
class DetectTagsInROSImage
{
public:
  /**
   * @brief this will detect tags of the given tag family
   * @param tagCodes
   */
  DetectTagsInROSImage(const AprilTags::TagCodes &tagCodes);

  /**
   * @brief detect tags in the given image
   * @param ros_img
   * @throw std::runtime_error if there is a problem with image processing
   */
  std::vector<AprilTags::TagDetection> operator()(const sensor_msgs::ImageConstPtr &ros_img);
private:
  AprilTags::TagDetector tag_detector_;
};
}

#endif // APRIL_TAG_DETECT_IN_ROS_IMAGE_H
