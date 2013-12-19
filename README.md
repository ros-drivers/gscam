GSCam
=====

This is a ROS package originally developed by the [Brown Robotics
Lab](http://robotics.cs.brown.edu/) for broadcasting any
[GStreamer](http://gstreamer.freedesktop.org/)-based video stream via the
standard [ROS Camera API](http://ros.org/wiki/camera_drivers). This fork has
several fixes incorporated into it to make it broadcast correct
`sensor_msgs/Image` messages with proper frames and timestamps. It also allows
for more ROS-like configuration and more control over the GStreamer interface.

Note that this pacakge can be built both in a rosbuild and catkin workspaces.

ROS API (stable)
----------------

### gscam

This can be run as both a node and a nodelet.

#### Nodes
 * `gscam`

#### Topics
 * `camera/image_raw`
 * `camera/camera_info`

#### Services
 * `camera/set_camera_info`

#### Parameters
 * `~camera_name`: The name of the camera (corrsponding to the camera info)
 * `~camera_info_url`: A url (`file://path/to/file`, `package://pkg_name/path/to/file`) to the [camera calibration file](http://www.ros.org/wiki/camera_calibration_parsers#File_formats).
 * `~gscam_config`: The GStreamer [configuration string](http://wiki.oz9aec.net/index.php?title=Gstreamer_cheat_sheet&oldid=1829).
 * `~frame_id`: The [TF](http://www.ros.org/wiki/tf) frame ID.
 * `~reopen_on_eof`: Re-open the stream if it ends (EOF).
 * `~sync_sink`: Synchronize the app sink (sometimes setting this to `false` can resolve problems with sub-par framerates).

C++ API (unstable)
------------------

The gscam c++ library can be used, but it is not guaranteed to be stable. 

Examples
--------

See example launchfiles and configs in the examples directory. Currently there
are examples for:
 * [Video4Linux2](examples/v4l.launch): Standard
   [video4linux](http://en.wikipedia.org/wiki/Video4Linux)-based cameras like
   USB webcams
 * [Nodelet](examples/gscam_nodelet.launch): Run a V4L-based camera in a nodelet
 * [Video File](examples/videofile.launch): Any videofile readable by GStreamer
 * [DeckLink](examples/decklink.launch):
   [BlackMagic](http://www.blackmagicdesign.com/products/decklink/models)
   DeckLink SDI capture cards (note: this requires the `gst-plugins-bad` plugins)
