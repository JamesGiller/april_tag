#include "april_tag_ros/visualize_tags.h"

#include <algorithm>

#include <visualization_msgs/MarkerArray.h>

#include <boost/bind.hpp>

namespace april_tag
{
VisualizeTags::VisualizeTags(ros::NodeHandle &nh)
{
  double tag_size_cm;
  if(!nh.getParam("tag_size_cm", tag_size_cm))
  {
    throw std::runtime_error{"Parameter 'tag_size_cm' is required but not set"};
  }
  const double tag_size_m = tag_size_cm / 100.0;

  auto callback = boost::bind(&VisualizeTags::publishMarkersForTags_, this, _1, nh.getNamespace(), tag_size_m);

  tag_marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_tag_array", 1, true);
  tag_list_subscriber_ = nh.subscribe<AprilTagList>("/april_tags", 1, callback);
}

void VisualizeTags::publishMarkersForTags_(const AprilTagListConstPtr &list, std::string marker_ns, double tag_size_m)
{
  const double thickness = 0.005;

  visualization_msgs::MarkerArray tag_markers;
  std::transform(list->april_tags.begin(), list->april_tags.end(), std::back_inserter(tag_markers.markers),
                 [&, this](const AprilTag &tag) -> visualization_msgs::Marker {
    const double thickness = 0.005;

    visualization_msgs::Marker marker;
    marker.header = tag.stamped.header;
    marker.ns = marker_ns;
    marker.id = tag.id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = tag.stamped.pose;
    marker.pose.position.z -= thickness / 2.0;
    marker.scale.x = tag_size_m;
    marker.scale.y = marker.scale.x;
    marker.scale.z = thickness;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration{1};

    return marker;
  });

  tag_marker_publisher_.publish(tag_markers);
}
}
