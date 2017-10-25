/** \author James Giller */

#ifndef APRIL_TAG_APRIL_TAG_NODELET_H
#define APRIL_TAG_APRIL_TAG_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>
#include <ros/timer.h>
#include <sensor_msgs/CameraInfo.h>

#include "april_tag_ros/tag_publisher.h"

namespace april_tag
{
class AprilTagNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() override;

private:
  std::unique_ptr<TagPublisher> tag_publisher_;
};
}
#endif // APRIL_TAG_APRIL_TAG_NODELET_H
