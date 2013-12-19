^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gscam
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
