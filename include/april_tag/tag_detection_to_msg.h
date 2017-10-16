/** \author James Giller */

#ifndef APRIL_TAG_TAG_DETECTION_TO_MSG_H
#define APRIL_TAG_TAG_DETECTION_TO_MSG_H

#include <sensor_msgs/CameraInfo.h>

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

  /**
   * @brief conveniently set all four values at once
   * @param focal_length_x_px
   * @param focal_length_y_px
   * @param principal_point_x_px
   * @param principal_point_y_px
   */
  inline void setCameraIntrinsics(double focal_length_x_px, double focal_length_y_px,
                                  double principal_point_x_px, double principal_point_y_px)
  {
    camera_focal_length_x_px = focal_length_x_px;
    camera_focal_length_y_px = focal_length_y_px;
    camera_principal_point_x_px = principal_point_x_px;
    camera_principal_point_y_px = principal_point_y_px;
  }

  /**
   * @brief use a CameraInfo msg to set all four values
   *
   * Convenient because sensor_msgs/CameraInfo only describes contents of intrinsic camera matrix in msg file comments
   * @param camera_info
   */
  inline void setIntrinsicsFromCameraInfo(const sensor_msgs::CameraInfo &camera_info)
  {
    setCameraIntrinsics(camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]);
  }
};



/**
 * @brief generate an AprilTag msg given a TagDetection and metadata
 * @param detection describes the detection of a tag in an image
 * @param context describes the context in which tag was detected
 */
AprilTag tagDetectionToMsg(const AprilTags::TagDetection &detection, const DetectionContext &context);
}

#endif // APRIL_TAG_TAG_DETECTION_TO_MSG_H
