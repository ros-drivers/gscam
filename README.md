GSCam
=====

This is a ROS package originally developed by the [Brown Robotics Lab](http://robotics.cs.brown.edu/) for broadcasting any [Gstreamer](http://gstreamer.freedesktop.org/)-based video stream via the standard [ROS Camera API](http://ros.org/wiki/camera_drivers). This fork has several fixes incorporated into it to make it broadcast correct sensor_msgs/Image messages with proper frames and timestamps. It also allows for more ROS-like configuration and more control over the GStreamer interface.

Examples
--------

See example launchfiles and configs in the examples directory. Currently there are examples for:
 * [Video4Linux2](examples/v4l.launch): Standard [video4linux](http://en.wikipedia.org/wiki/Video4Linux)-based cameras like USB webcams
 * [DeckLink](examples/decklink.launch): [BlackMagic](http://www.blackmagicdesign.com/products/decklink/models) DeckLink SDI capture cards
