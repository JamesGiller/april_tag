/** \author James Giller */
/** \author palash */

#include "april_tag/detect_tags_in_ros_image.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace april_tag
{
DetectTagsInROSImage::DetectTagsInROSImage(const AprilTags::TagCodes &tagCodes) : tag_detector_{tagCodes}
{

}

std::vector<AprilTags::TagDetection> DetectTagsInROSImage::operator ()(const sensor_msgs::ImageConstPtr &ros_img)
{
  try
  {
    auto cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat image_gray;
    cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
    return tag_detector_.extractTags(image_gray);
  }
  catch (cv_bridge::Exception& e)
  {
    throw std::runtime_error{e.what()};
  }
}
}
