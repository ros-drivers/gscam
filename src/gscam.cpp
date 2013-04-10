
#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>


#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <camera_calibration_parsers/parse_ini.h>

#include <gscam/gscam.h>

namespace gscam {

  GSCam::GSCam(ros::NodeHandle nh_camera, ros::NodeHandle nh_private) : 
    gsconfig_(""),
    pipeline_(NULL),
    sink_(NULL),
    nh_(nh_camera),
    nh_private_(nh_private),
    image_transport_(nh_camera),
    camera_info_manager_(nh_camera)
  {
  }

  GSCam::~GSCam()
  {
  }

  bool GSCam::configure()
  {
    // Get gstreamer configuration
    // (either from environment variable or ROS param)
    std::string gsconfig_rosparam = "";
    bool gsconfig_rosparam_defined = false;
    char *gsconfig_env = NULL;

    gsconfig_rosparam_defined = nh_private_.getParam("gscam_config",gsconfig_rosparam);
    gsconfig_env = getenv("GSCAM_CONFIG");

    if (!gsconfig_env && !gsconfig_rosparam_defined) {
      ROS_FATAL( "Problem getting GSCAM_CONFIG environment variable and 'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline." );
      return false;
    } else if(gsconfig_env && gsconfig_rosparam_defined) {
      ROS_FATAL( "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. Please only define one." );
      return false;
    } else if(gsconfig_env) {
      gsconfig_ = gsconfig_env;
      ROS_INFO_STREAM("Using gstreamer config from env: \""<<gsconfig_env<<"\"");
    } else if(gsconfig_rosparam_defined) {
      gsconfig_ = gsconfig_rosparam;
      ROS_INFO_STREAM("Using gstreamer config from rosparam: \""<<gsconfig_rosparam<<"\"");
    }

    // Get additional gscam configuration
    nh_private_.param("sync_sink", sync_sink_, true);
    nh_private_.param("preroll", preroll_, false);

    nh_private_.param("reopen_on_eof", reopen_on_eof_, false);

    // Get the camera parameters file
    nh_private_.getParam("camera_info_url", camera_info_url_);
    nh_private_.getParam("camera_name", camera_name_);

    camera_info_manager_.setCameraName(camera_name_);

    if(camera_info_manager_.validateURL(camera_info_url_)) {
      camera_info_manager_.loadCameraInfo(camera_info_url_); 
      ROS_INFO_STREAM("Loaded camera calibration from "<<camera_info_url_);
    } else {
      ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
    }

    // Get TF Frame
    if(!nh_private_.getParam("frame_id",frame_id_)){
      frame_id_ = "/camera_frame";
      ROS_WARN_STREAM("No camera frame_id set, using frame \""<<frame_id_<<"\".");
      nh_private_.setParam("frame_id",frame_id_);
    }

    return true;
  }

  bool GSCam::init_stream()
  {
    if(!gst_is_initialized()) {
      // Initialize gstreamer pipeline
      ROS_DEBUG_STREAM( "Initializing gstreamer..." );
      gst_init(0,0);
    }

    ROS_DEBUG_STREAM( "Gstreamer Version: " << gst_version_string() );

    GError *error = 0; // Assignment to zero is a gst requirement

    pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
    if (pipeline_ == NULL) {
      ROS_FATAL_STREAM( error->message );
      return false;
    }

    // Create RGB sink
    sink_ = gst_element_factory_make("appsink",NULL);
    GstCaps * caps = gst_caps_new_simple("video/x-raw-rgb", NULL);
    gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
    gst_caps_unref(caps);

    // Set whether the sink should sync
    // Sometimes setting this to true can cause a large number of frames to be
    // dropped
    gst_base_sink_set_sync(
        GST_BASE_SINK(sink_), 
        (sync_sink_) ? TRUE : FALSE);

    if(GST_IS_PIPELINE(pipeline_)) {
      GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
      g_assert(outpad);

      GstElement *outelement = gst_pad_get_parent_element(outpad);
      g_assert(outelement);
      gst_object_unref(outpad);

      if(!gst_bin_add(GST_BIN(pipeline_), sink_)) {
        ROS_FATAL("gst_bin_add() failed");
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      if(!gst_element_link(outelement, sink_)) {
        ROS_FATAL("GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      gst_object_unref(outelement);
    } else {
      GstElement* launchpipe = pipeline_;
      pipeline_ = gst_pipeline_new(NULL);
      g_assert(pipeline_);

      gst_object_unparent(GST_OBJECT(launchpipe));

      gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

      if(!gst_element_link(launchpipe, sink_)) {
        ROS_FATAL("GStreamer: cannot link launchpipe -> sink");
        gst_object_unref(pipeline_);
        return false;
      }
    }

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
      return false;
    } else {
      ROS_DEBUG_STREAM("Stream is PAUSED.");
    }

    // Create ROS camera interface
    camera_pub_ = image_transport_.advertiseCamera("camera/image_raw", 1);

    return true;
  }

  void GSCam::publish_stream() 
  {
    ROS_INFO_STREAM("Publishing stream...");

    // Pre-roll camera if needed
    if (preroll_) {
      ROS_DEBUG("Performing preroll...");

      //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
      //I am told this is needed and am erring on the side of caution.
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PLAY during preroll.");
        return;
      } else {
        ROS_DEBUG("Stream is PLAYING in preroll.");
      }

      gst_element_set_state(pipeline_, GST_STATE_PAUSED);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PAUSE.");
        return;
      } else {
        ROS_INFO("Stream is PAUSED in preroll.");
      }
    }

    if(gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Could not start stream!");
      return;
    }
    ROS_INFO("Started stream.");

    // Poll the data as fast a spossible
    while(ros::ok()) {
      // This should block until a new frame is awake, this way, we'll run at the 
      // actual capture framerate of the device.
      ROS_DEBUG("Getting data...");
      GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink_));


      GstFormat fmt = GST_FORMAT_TIME;
      gint64 current = -1;

      // Query the current position of the stream
      //if (gst_element_query_position(pipeline_, &fmt, &current)) {
        //ROS_INFO_STREAM("Position "<<current);
      //}

      // Stop on end of stream
      if (!buf) {
        ROS_INFO("Stream ended.");
        break;
      }

      ROS_DEBUG("Got data.");

      // Get the image width and height
      GstPad* pad = gst_element_get_static_pad(sink_, "sink");
      const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
      GstStructure *structure = gst_caps_get_structure(caps,0);
      gst_structure_get_int(structure,"width",&width_);
      gst_structure_get_int(structure,"height",&height_);

      // Complain if the returned buffer is smaller than we expect
      if (buf->size < unsigned(width_ * height_ * 3)) {
        ROS_WARN_STREAM( "GStreamer image buffer underflow: Expected frame to be "
            << (width_ * height_ * 3) << " bytes but got only "
            << (buf->size) << " bytes. (make sure frames are raw RGB encoded)");
      }

      // Construct Image message
      sensor_msgs::ImagePtr img(new sensor_msgs::Image());
      sensor_msgs::CameraInfoPtr cinfo;

      // Update header information
      cinfo.reset(new sensor_msgs::CameraInfo(camera_info_manager_.getCameraInfo()));
      cinfo->header.stamp = ros::Time::now();
      cinfo->header.frame_id = frame_id_;
      img->header = cinfo->header;

      // Image data and metadata
      img->width = width_; 
      img->height = height_;
      img->encoding = "rgb8";
      img->is_bigendian = false;
      img->step = width_*3;
      img->data.resize(width_*height_*3);

      // Copy only the data we received
      // Since we're publishing shared pointers, we need to copy the image so
      // we can free the buffer allocated by gstreamer
      std::copy(
          buf->data,
          (buf->data)+(buf->size),
          img->data.begin());

      // Publish the image/info
      camera_pub_.publish(img, cinfo);

      // Release the buffer
      gst_buffer_unref(buf);

      ros::spinOnce();
    }
  }

  void GSCam::cleanup_stream()
  {
    // Clean up
    ROS_INFO("Stopping gstreamer pipeline...");
    if(pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = NULL;
    }
  }

  void GSCam::run() {
    while(ros::ok()) {
      if(!this->configure()) {
        ROS_FATAL("Failed to configure gscam!");
        break;
      }

      if(!this->init_stream()) {
        ROS_FATAL("Failed to initialize gscam stream!");
        break;
      }

      // Block while publishing
      this->publish_stream();

      this->cleanup_stream();

      ROS_INFO("GStreamer stream stopped!");

      if(reopen_on_eof_) {
        ROS_INFO("Reopening stream...");
      } else {
        ROS_INFO("Cleaning up stream and exiting...");
        break;
      }
    }

  }

  // Example callbacks for appsink
  // TODO: enable callback-based capture
  void gst_eos_cb(GstAppSink *appsink, gpointer user_data ) {
  }
  GstFlowReturn gst_new_preroll_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }
  GstFlowReturn gst_new_asample_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }

}
