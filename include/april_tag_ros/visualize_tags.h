/** \author James Giller */
#ifndef APRIL_TAG_VISUALIZE_TAGS_H
#define APRIL_TAG_VISUALIZE_TAGS_H

#include <ros/node_handle.h>

#include "april_tag/AprilTagList.h"

namespace april_tag
{
/**
 * @brief Publishes visualization_msgs/MarkerArray msgs containing all tags detected in any one image
 */
class VisualizeTags
{
public:
  /**
   * @brief this will publish MarkerArrays to 'visualize_tag_array' topic
   * @param nh Markers will share the namespace of the node
   */
  VisualizeTags(ros::NodeHandle &nh);

private:
  void publishMarkersForTags_(const AprilTagListConstPtr &list, std::string marker_ns, double tag_size_m);

  ros::Publisher tag_marker_publisher_;
  ros::Subscriber tag_list_subscriber_;
};
}

#endif // APRIL_TAG_VISUALIZE_TAGS_H
