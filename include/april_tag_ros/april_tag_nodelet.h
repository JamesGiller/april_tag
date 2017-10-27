/** \author James Giller */

#ifndef APRIL_TAG_APRIL_TAG_NODELET_H
#define APRIL_TAG_APRIL_TAG_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>
#include <ros/timer.h>
#include <sensor_msgs/CameraInfo.h>

#include "april_tag_ros/tag_publisher.h"
#include "april_tag_ros/visualize_tags.h"

namespace april_tag
{
class AprilTagNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() override;

private:
  std::unique_ptr<TagPublisher> tag_publisher_;
  std::unique_ptr<VisualizeTags> visualize_tags_;
  ros::Timer initialization_thread_;
};
}
#endif // APRIL_TAG_APRIL_TAG_NODELET_H
