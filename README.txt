------------------------------------------------------------
------------------------------------------------------------
Overview:
------------------------------------------------------------
	gscam is meant as a simple approach to using a webcam in ROS that
maximizes compatibility. gscam leverages Gstreamer [ http://www.gstreamer.net/
], a multimedia framework similiar to DirectShow. Specifically: Gstreamer can be
used to build up multimedia "pipelines" consisting of sources, sinks, and
filters. For example: a v4l webcam *source* might be *filtered* by an upscaler
before being sent to the screen (a *sink*). Alternately, a mp4 file might act as
a source and be filtered into an avi file as a sink. There are many
possibilities. For an overview see: ["Using Gstreamer"
http://gstreamer.freedesktop.org/data/doc/gstreamer/head/faq/html/chapter-using.html
]. gscam can attach itself to an identity filter within any Gstreamer pipeline
that gscam itself launches. Provided this pipeline is processing RGB video,
gscam will rebroadcast the video over a standard ROS image transport. Since
Gstreamer is compatibile with almost every video capture standard under Linux
(and many on OSX), gscam makes ROS defacto compatible with almost every Linux
webcam or video system available. Moreover, because there are a number of
Gstreamer filters for processing video (e.g. white-balancing, cropping, etc.)
gscam allows for a more advanced video processing even with cheaper webcams.

	Gstreamer is an extremely rich framework, but basic usage is fairly
simple. Please see the "Basic Usage" section to get started. You may also find
the Gstreamer documentation on their website to be helpful.

------------------------------------------------------------
------------------------------------------------------------
Requirements:
------------------------------------------------------------
	Obviously, gscam requires the Gstreamer libraries and include files to
be available in order to build. Additionally, most operating systems tend to
make available various seperate packages of Gstreamer "plugins" (sources, sinks,
and filters). The more of these the merrier, as each adds a unique feature
possibility to the available pipelines.

------------------------------------------------------------
------------------------------------------------------------
Basic Usage:
------------------------------------------------------------

gscam expects an enviromental variable, GSCAM_CONFIG, to contain a gstreamer
pipeline definition for it to launch. There are two restrictions:

1) At some point in the pipeline there *must* be an identity filter named ros .
2) At the point in the pipeline where the ros identity filter is attached, the
pipeline *must* be carrying RGB formatted video.

Here's an example launch that should work on almost any machine where Gstreamer
is installed and a camera of some kind is available:

export GSCAM_CONFIG="v4l2src device=/dev/video2 ! video/x-raw-rgb ! ffmpegcolorspace ! identity name=ros ! fakesink"
rosrun gscam gscam

If you see a message about a "failure to PAUSE" or something similiar, the most
likely cause is a permission issue with the available video devices.

If you see a "Processing..." message, it means gscam is happily publising to
the /gscam/image_raw image transport.

You can graphically explore many of the available video devices with the
gstreamer-properties utility. gst-inspect-0.10 (or .09 or whatever is your
Gstreamer version number) will list all of the available plugins.
