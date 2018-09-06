^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gscam
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2018-09-06)
------------------
* Merge pull request `#52 <https://github.com/ros-drivers/gscam/issues/52>`_ from k-okada/add_travis
  update travis.yml
* Update README.md
  gscam -> GSCam
* update travis.yml
* Merge pull request `#40 <https://github.com/ros-drivers/gscam/issues/40>`_ from ros-drivers/mikaelarguedas-patch-1
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* Contributors: Kei Okada, Mikael Arguedas

1.0.0 (2017-06-13)
------------------
* Gstreamer 1 0 support (`#36 <https://github.com/ros-drivers/gscam/issues/36>`_ )
  * delete unused manifest.xml, rosdep.yaml
  * use libgstreamer1.0 for lunar
  * Fixing preproc switches so that the 1.0 branch still builds against 0.1 without the switch flags
  * Preliminary GStreamer-1.0 support see README for more info
* Contributors: Jonathan Bohren, Kei Okada

0.2.0 (2017-06-13)
------------------
* add ROS Orphaned Package Maintainers to maintainer tag (`#35 <https://github.com/ros-drivers/gscam/issues/35>`_ )
* gscam_nodelet.h: include scoped_ptr.hpp to compile with boost 1.57
  When compiling gscam with the currently latest boost version 1.57,
  it fails with:
  In file included from [...]/src/gscam_nodelet.cpp:5:0:
  [...]/include/gscam/gscam_nodelet.h:20:12: error: 'scoped_ptr' in namespace 'boost' does not name a template type
  boost::scoped_ptr<GSCam> gscam_driver\_;
  ^
  [...]/include/gscam/gscam_nodelet.h:21:12: error: 'scoped_ptr' in namespace 'boost' does not name a template type
  boost::scoped_ptr<boost::thread> stream_thread\_;
  ^
  It seems that the dependencies of boost/thread.hpp have changed
  and boost/scoped_ptr.hpp is not included anymore with
  boost/thread.hpp. Hence, the scoped_ptr is not defined in the
  gscam_nodelet header. After scanning quickly through the release
  notes of version 1.57, the boost bug tracker and a few changesets,
  I could not find not a hint what has changed in the thread library
  that gscam must include scoped_ptr itself.
  This commit simply addresses the compiler error by explicitly
  adding boost's scoped_ptr header in the gscam_nodelet header.
  As this commit also compiles with boost version 1.56, the commit
  is not expected to cause any problems with other build
  configurations.
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Remove dependency on opencv2 to fix under indigo
  Packages can no longer depend on opencv2 as of indigo.
  I've updated the package to depend instead on cv_bridge as suggested by http://wiki.ros.org/indigo/Migration#OpenCV.
* Install the parameters file refered to in v4l.launch
* Update package.xml
* Examples: Added example for OSX (`#15 <https://github.com/ros-drivers/gscam/issues/15>`_)
  Add a simple launch configuration for OSX. The camera can be selected by
  changing the default="0" to the appropriate integer.
* Adding libraries to gscam target
  Fixes `#13 <https://github.com/ros-drivers/gscam/issues/13>`_, now builds on stricter linkers
* adding proper depends to catkin package call
* Update minoru.launch
* Contributors: CHILI Demo Corner, Jonathan Bohren, Kei Okada, Kenn Sebesta, Lukas Bulwahn, Russell Toris, Lukas Bulwahn

0.1.3 (2013-12-19)
------------------
* Removed special characters from changelog.

0.1.2 (2013-12-19)
------------------
* Added install targets for headers
* Adding note on blackmagic decklink cards
* Make sure nodelets are usable
* Add jpeg direct publishing without decoding
* Added system to use GST timestamps
* Valid test file for jpeg-based publisher
* Prepared for jpeg-only subscription
* Install some launch files + added nodelet pipeline demo
* Added missing bits to install the nodelet
* Support for gray (mono8) cameras.
* Remove unused ``bpp_`` member.
* Expose default settings for ``image_encodings`` to the ros master.
* Add in support for mono cameras.
* Contributors: Cedric Pradalier, Holger Rapp, Jonathan Bohren, Russell Toris, Severin Lemaignan

0.1.1 (2013-05-30)
------------------
* adding missing nodelet dep
* Contributors: Jonathan Bohren

0.1.0 (2013-05-28)
------------------
* adding maintainer/authors
* making node name backwards compatible
* re-adding package.xml
* more info spam
* removing old camera parameters file
* updating gscam to use ``camera_info_manager``
* Fixing nodelet, adding example
* Making gscam a node and nodeelt
* adding a note on the videofile player and wrapping readme
* adding option to reopen a stream on EOF and adding a videofile example
* Hybrid catkin-rosbuild buildsystem
* adding minoru example
* putting example nodes into namespaces, adding correct error check in gscam source, making tf frame publishing optional
* rst->md
* making gscam conform to standard ``camera_drivers`` ROS API, note, still need to add polled mode
* fixes for decklink capture, adding another example
* can't have manifest.xml and package.xml in same directory
* removing unneeded find-pkgs
* building in catkin ws
* hybrid rosbuild/catkin buildsystem
* Adding changes that were made to the distribution branch that
  should have gone into the exerpeimental branch in r2862.
  Added a bunch of enhancements and fixed bugs involving data
  missing fromthe image message headers.
  Index: src/gscam.cpp
  ===================================================================
  Added ``camera_name`` and ``camera_parameters_file`` globals for camera
  info.
  Moved ros init to the top of the main function.
  Gets the gstreamer configuration either from environment variable
  ``GSCAM_CONFIG`` or ROS param ``~/gscam_config``.
  Gets the camera calibration parameters from the file located at ROS
  param ``~/camera_parameters_file``, will look at
  "../camera_settings.txt" by default.
  A bunch of re-indenting for consistency.
  Updated a lot of error fprintfs to ``ROS_ERROR`` calls.
  Gets the TF ``frame_id`` from the ROS param ``~/frame_id``, can be over-
  written by camera parameters.
  Now sets the appropriate ROS timestap in the image message header.
  Now sets the appropriate TF frame in the image message header.
  Added more detailed info/error/warn messages.
  Modified the warning / segfault avoidance added to experimental in
  r2756. Instead of skipping the frame, it just copies only the
  amount of data that it was received, and reports the warning each
  time, instead of just once. In a large scale system with lots of
  messages, a single warning might easily get lost in the noise.
  Index: examples/webcam_parameters.txt
  ===================================================================
  Added example camera parameters (uncalibrated) for a laptop webcam.
  Index: examples/webcam.launch
  ===================================================================
  Added a launchfile that makes use of the new rosparam options and
  TF frame.
* avoid segfault when buffer size is too small
* ROSProcessingjs clean-up
* makefile so rosmake is more reliable
* gscam build tweak for oneiric
* fixes for Natty build per Willow request
* stop node on EOS
* File support courtesy of John Hoare of the University of Tennesse at Knoxville
* more conservative license policy
* fps workaround
* ding gscam
* back to before
* two publishers
* Lots of changes.  AR Alpha now expects files in the bin directory, to facilitate roslaunch.  Gscam must be started from the bin directory, or, again, using roslaunch.  The localizer code now works correctly and has been tested on a Create, but has problems cause by AR alpha's processing delays.
* Bugfix: supply default camera parameters when real ones are unavailable.
* Fully-functional calibration file writing.
* Partial changes for file-writing gscam.
* Gscam now fits into an image processing pipeline with rectified images.  TODO: Save camera configuration info.
* Handles built for camera info services, but no testing.
* Changed the name of the GStreamer camera package.  probe will henceforth be known as gscam.
* Contributors: Jonathan Bohren, chriscrick, evan.exe@gmail.com, nevernim@gmail.com, trevorjay
