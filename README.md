april_tag
=========

Detects 'AprilTag' 2D fiducial markers in ROS images (`sensor_msgs/Image`) and produces id, location, and orientation of the tags. Functionality is also provided as a Nodelet.

This package uses the C++ AprilTags library written by Michael Kaess and Hordur Johannson. The AprilTags Visual Fiducial System was developed by Edwin Olson.

More on april tags here:
https://april.eecs.umich.edu/software/apriltag.html

April Tags C++ library:
http://people.csail.mit.edu/kaess/apriltags/

## Launchfiles

- april\_tag.launch
- nodelet.launch

### Required arguments
- camera\_ns: namespace in which camera's image\_raw topic exists e.g. /usb\_cam
- tag\_size\_cm: length of the edge of the black frame in cms
- manager: name of nodelet manager (_nodelet only_)

### Optional arguments
- focal\_length\_x\_px: camera focal length in x dimension in pixels
- focal\_length\_y\_px: camera focal length in y dimension in pixels
- principal\_point\_x\_px: x-coordinate of camera principal point
- principal\_point\_y\_px: y-coordinate of camera principal point

Missing optional arguments are fetched from `sensor_msgs/CameraInfo` messages published on the `$(arg camera_ns)/camera_info` topic.

## Input

`$(arg camera_ns)/image_raw`

## Output

AprilTagList which is a list of AprilTag:

```
uint32 	id
uint32 	hamming_distance
float64 distance
geometry_msgs/PoseStamped stamped
```

where hamming\_distance indicates difference between the observed tag and the identified tag, and distance is depth away from camera. x is horizontal with camera right as positive. 

Depends on: libeigen3-dev

-- James Giller







