# GSCam ![ROS2 CI](https://github.com/ros-drivers/gscam/actions/workflows/build_test.yml/badge.svg?branch=ros2)

This is a ROS2 package originally developed by the [Brown Robotics
Lab](http://robotics.cs.brown.edu/) for broadcasting any
[GStreamer](http://gstreamer.freedesktop.org) video stream via
[image transport](https://index.ros.org/p/image_transport/).

## GStreamer Library Support

GSCam supports the following versions of ROS2 and GStreamer:

| ROS2 version | Ubuntu version | GStreamer version |
|---|---|---|
| Foxy | 20.04 | 1.16 |
| Galactic | 20.04 | 1.16 |
| Humble | 22.04 | 1.20 |
| Rolling | 22.04 | 1.20 |

#### Dependencies

These dependencies will be picked up by rosdep and are required to compile:
* libgstreamer1.0-dev 
* libgstreamer-plugins-base1.0-dev 

These additional packages are often useful:
* gstreamer1.0-tools
* libgstreamer-plugins-good1.0-dev

Ubuntu install:

```sh
sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
```

## License

GSCam for ROS2 is licensed under the [Apache 2 license](LICENSE).

Note that GStreamer is licensed under the LGPL, and GStreamer plugins have their own 
[license requirements](https://gstreamer.freedesktop.org/documentation/plugin-development/appendix/licensing-advisory.html?gi-language=c).

## API

#### Nodes
* `gscam`

#### Topics
* `camera/image_raw`
* `camera/camera_info`

#### Services
* `camera/set_camera_info`

#### Parameters
* `camera_name`: The name of the camera (corrsponding to the camera info)
* `camera_info_url`: A url (`file://path/to/file`, `package://pkg_name/path/to/file`) to the 
  [camera calibration file](http://www.ros.org/wiki/camera_calibration_parsers#File_formats)
* `gscam_config`: The GStreamer [configuration string](https://gstreamer.freedesktop.org/documentation/tools/gst-launch.html?gi-language=c#pipeline-examples)
* `frame_id`: The [tf2](https://index.ros.org/p/tf2/) frame ID
* `reopen_on_eof`: Re-open the stream if it ends (EOF)
* `sync_sink`: Synchronize the app sink (sometimes setting this to `false` can resolve problems with sub-par framerates)
* `use_gst_timestamps`: Use the GStreamer buffer timestamps for the image message header timestamps (setting this to `false` results in header timestamps being the time that the image buffer transfer is completed)
* `image_encoding`: image encoding ("rgb8", "mono8", "yuv422", "jpeg")
* `use_sensor_data_qos`: The flag to use sensor data qos for camera topic(image, camera_info)

## Examples

See example launch files and configs in the [examples](examples) directory.

These examples have been tested and work well:
* [v4l.launch.xml](examples/v4l.launch.xml): Standard
  [video4linux](http://en.wikipedia.org/wiki/Video4Linux) cameras like USB webcams
* [v4ljpeg.launch.xml](examples/v4ljpeg.launch.xml): Same as above, but publishes 
  [compressed images](https://docs.ros2.org/foxy/api/sensor_msgs/msg/CompressedImage.html)
* [videofile.launch.xml](examples/videofile.launch.xml): Opens any videofile readable by GStreamer
* [component_pipeline_launch.py](examples/component_pipeline_launch.py): Launch an image pipeline using [ROS2 composition](https://docs.ros.org/en/foxy/Tutorials/Composition.html)
