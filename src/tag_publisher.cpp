/** \author James Giller */
/** \author palash */

#include "april_tag_ros/tag_publisher.h"

#include <algorithm>
#include <iterator>

#include <boost/format.hpp>

namespace april_tag
{
TagPublisher::TagPublisher(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const AprilTags::TagCodes &tag_codes,
                           TagPublisher::Loggers loggers) :
  detect_tags_{tag_codes},
  it_{nh},
  loggers_{std::move(loggers)}
{
  using namespace boost;

  double tag_size_cm;
  if(!nh.getParam("tag_size_cm", tag_size_cm))
  {
    throw std::runtime_error{"Parameter 'tag_size_cm' is required but not set"};
  }
  detection_context_.tag_size_m = tag_size_cm / 100.0;

  std::string camera_ns;
  std::string image_topic;
  if(!private_nh.getParam("camera_ns", camera_ns))
  {
    throw std::runtime_error{"Parameter 'camera_ns' is required but not set"};
  }
  if(!private_nh.getParam("image_topic", image_topic))
  {
    throw std::runtime_error{"Parameter 'image_topic' is required but not set"};
  }

  double fx, fy, cx, cy;
  private_nh.param<double>("focal_length_x_px", fx, -1.);
  private_nh.param<double>("focal_length_y_px", fy, -1.);
  private_nh.param<double>("principal_point_x_px", cx, -1.);
  private_nh.param<double>("principal_point_y_px", cy, -1.);

  if(fx > 0. && fy > 0. && cx > 0. && cy > 0.)
  {
    detection_context_.setCameraIntrinsics(fx, fy, cx, cy);
  }
  else
  {
    loggers_.log_info(str(format{"Camera intrinsics not supplied. Waiting to fetch from topic '%s/camera_info'"}
                          % camera_ns.c_str()));
    const auto camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_ns + "/camera_info", nh);
    if(!camera_info)
    {
      throw std::runtime_error{"Interrupted by node shutdown"};
    }
    if(std::all_of(camera_info->K.begin(), camera_info->K.end(), [this](double k) { return k == 0;}))
    {
      loggers_.log_warn(str(format{"Camera intrinsics fetched from topic '%s/camera_info' are all zero."}
                            % camera_ns.c_str()));
    }
    detection_context_.setIntrinsicsFromCameraInfo(*camera_info);
  }

  tag_list_pub_ = nh.advertise<AprilTagList>("/april_tags", 100);
  image_sub_ = it_.subscribe(camera_ns + image_topic, 1, &TagPublisher::detectAndPublishTags_, this);
}

void TagPublisher::detectAndPublishTags_(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    auto detections = detect_tags_(msg);

    if(detections.size() > 0)
    {
      detection_context_.time_and_place = msg->header;
      AprilTagListPtr tag_list{new AprilTagList};
      std::transform(detections.begin(), detections.end(), std::back_inserter(tag_list->april_tags),
                     std::bind(tagDetectionToMsg, std::placeholders::_1, detection_context_));
      tag_list_pub_.publish(tag_list);
    }
  }
  catch (const std::runtime_error& e)
  {
    loggers_.log_error(std::string{e.what()});
  }
}
}
