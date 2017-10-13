/** \author James Giller */

#ifndef APRIL_TAG_TAG_DETECTION_TO_MSG_H
#define APRIL_TAG_TAG_DETECTION_TO_MSG_H

#include <std_msgs/Header.h>

#include "april_tag/AprilTag.h"
#include "AprilTags/TagDetection.h"

namespace april_tag
{
/**
 * @brief Metadata that describes the context in which a tag was detected
 */
struct DetectionContext
{
  std_msgs::Header time_and_place; ///< Time of detection, and frame id of camera that captured the image
  double camera_focal_length_x_px; ///< Value of fx of camera intrinsics matrix
  double camera_focal_length_y_px; ///< Value of fy of camera intrinsics matrix
  double camera_principal_point_x_px; ///< Value of cx of camera intrinsics matrix
  double camera_principal_point_y_px; ///< Value of cy of camera intrinsics matrix
  double tag_size_m; ///< Size of tags that are being used
};

/**
 * @brief generate an AprilTag msg given a TagDetection and metadata
 * @param detection describes the detection of a tag in an image
 * @param context describes the context in which tag was detected
 */
AprilTag tagDetectionToMsg(const AprilTags::TagDetection &detection, const DetectionContext &context);
}

#endif // APRIL_TAG_TAG_DETECTION_TO_MSG_H
