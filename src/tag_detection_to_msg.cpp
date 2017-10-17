/** \author James Giller */
/** \author palash */

#include "april_tag_ros/tag_detection_to_msg.h"

#include <cmath>

#include <boost/math/constants/constants.hpp>

#include <tf/transform_datatypes.h>

namespace
{
  /*
   * Normalize angle to be within the interval [-pi,pi].
   */
  inline double standardRad(double t)
  {
    using namespace boost::math::constants;

    if(t >= 0.)
    {
      return std::fmod(t + pi<double>(), two_pi<double>()) - pi<double>();
    }
    else
    {
      return std::fmod(t - pi<double>(), -two_pi<double>()) + pi<double>();
    }
  }

  /*
   * Convert rotation matrix to Euler angles
   */
  void rotationMatToEuler(const Eigen::Matrix3d &wRo, double &yaw, double &pitch, double &roll)
  {
    yaw = standardRad(std::atan2(wRo(1,0), wRo(0,0)));
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    pitch = standardRad(std::atan2(-wRo(2,0), wRo(0,0) * c + wRo(1,0) * s));
    roll  = standardRad(std::atan2(wRo(0,2) * s - wRo(1,2) * c, -wRo(0,1) * s + wRo(1,1) * c));
  }
}

namespace april_tag
{
AprilTag tagDetectionToMsg(const AprilTags::TagDetection &detection, const DetectionContext &context)
{
  const Eigen::Matrix4d transformation_matrix = detection.getRelativeTransform(context.tag_size_m,
                                                                               context.camera_focal_length_x_px,
                                                                               context.camera_focal_length_y_px,
                                                                               context.camera_principal_point_x_px,
                                                                               context.camera_principal_point_y_px);
  const Eigen::Vector3d translation = transformation_matrix.col(3).head(3);
  const Eigen::Matrix3d rotation = transformation_matrix.block(0, 0, 3, 3);

  Eigen::Matrix3d F;
  F <<
    1, 0,  0,
    0,  -1,  0,
    0,  0,  1;
  const Eigen::Matrix3d fixed_rot = F * rotation;

  april_tag::AprilTag tag_msg;
  tag_msg.id = detection.id;
  tag_msg.hamming_distance = detection.hammingDistance;
  tag_msg.distance = translation.norm();
  tag_msg.stamped.header.stamp = context.time_and_place.stamp;
  tag_msg.stamped.header.frame_id = context.time_and_place.frame_id;
  tag_msg.stamped.pose.position.x = translation(0);
  tag_msg.stamped.pose.position.y = translation(1);
  tag_msg.stamped.pose.position.z = translation(2);
  double roll, pitch, yaw;
  rotationMatToEuler(fixed_rot, yaw, pitch, roll);
  tag_msg.stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  return tag_msg;
}
}
