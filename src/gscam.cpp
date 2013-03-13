#include <stdlib.h>
#include <unistd.h>

#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>

#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>

//forward declarations
bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);

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

// Globals / camera configuration
int width, height;
sensor_msgs::CameraInfo camera_info;
std::string camera_name;
std::string camera_parameters_file;

void publish_stream(ros::NodeHandle &nh);

int main(int argc, char** argv) {
  ros::init(argc, argv, "gscam_publisher");
  ros::NodeHandle nh;

  bool reopen_on_eof;

  while(ros::ok()) {
    ros::param::param("~/reopen_on_eof", reopen_on_eof, false);

    publish_stream(nh);

    ROS_INFO("GStreamer stream stopped!");

    if(reopen_on_eof) {
      ROS_INFO("Reopening stream...");
    } else {
      ROS_INFO("Cleaning up stream and exiting...");
      break;
    }
  }

  return 0;
}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {

  ROS_INFO("New camera info received");
  camera_info = req.camera_info;

  if (camera_calibration_parsers::writeCalibrationIni(camera_parameters_file, camera_name, camera_info)) {
    ROS_INFO_STREAM("Camera calibration info written to \""<<camera_parameters_file<<"\" for camera \""<<camera_name<<"\".");
    return true;
  }
  else {
    ROS_ERROR_STREAM("Could not write \""<<camera_parameters_file<<"\"");
    return false;
  }
}

void publish_stream(ros::NodeHandle &nh) {

  // Get gstreamer configuration (either from environment variable or ROS param)
  std::string gsconfig = "";
  std::string gsconfig_rosparam = "";
  bool gsconfig_rosparam_defined = false;
  char *gsconfig_env = NULL;

  gsconfig_rosparam_defined = ros::param::get("~/gscam_config",gsconfig_rosparam);
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    ROS_FATAL( "Problem getting GSCAM_CONFIG environment variable and 'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline." );
    exit(-1);
  } else if(gsconfig_env && gsconfig_rosparam_defined) {
    ROS_FATAL( "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. Please only define one." );
    exit(-1);
  } else if(gsconfig_env) {
    gsconfig = gsconfig_env;
    ROS_INFO_STREAM("Using gstreamer config: \""<<gsconfig_env<<"\"");
  } else if(gsconfig_rosparam_defined) {
    gsconfig = gsconfig_rosparam;
    ROS_INFO_STREAM("Using gstreamer config: \""<<gsconfig_rosparam<<"\"");
  }

  // Initialize gstreamer pipeline
  gst_init(0,0);
  ROS_DEBUG_STREAM( "Gstreamer Version: " << gst_version_string() );

  GError *error = 0; //assignment to zero is a gst requirement
  GstElement *pipeline = gst_parse_launch(gsconfig.c_str(),&error);
  if (pipeline == NULL) {
    ROS_FATAL_STREAM( error->message );
    exit(-1);
  }
  GstElement * sink = gst_element_factory_make("appsink",NULL);
  GstCaps * caps = gst_caps_new_simple("video/x-raw-rgb", NULL);
  gst_app_sink_set_caps(GST_APP_SINK(sink), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be
  // dropped
  bool sync_sink;
  ros::param::param("~/sync_sink", sync_sink, true);
  gst_base_sink_set_sync(GST_BASE_SINK(sink), (sync_sink) ? TRUE : FALSE);

  if(GST_IS_PIPELINE(pipeline)) {
    GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline), GST_PAD_SRC);
    g_assert(outpad);
    GstElement *outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);


    if(!gst_bin_add(GST_BIN(pipeline), sink)) {
      ROS_FATAL("gst_bin_add() failed"); // TODO: do some unref
      gst_object_unref(outelement);
      gst_object_unref(pipeline);
      exit(-1);
    }

    if(!gst_element_link(outelement, sink)) {
      ROS_FATAL("GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline);
      exit(-1);
    }

    gst_object_unref(outelement);
  } else {
    GstElement* launchpipe = pipeline;
    pipeline = gst_pipeline_new(NULL);
    g_assert(pipeline);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline), launchpipe, sink, NULL);

    if(!gst_element_link(launchpipe, sink)) {
      ROS_FATAL("GStreamer: cannot link launchpipe -> sink");
      gst_object_unref(pipeline);
      exit(-1);
    }
  }

  gst_element_set_state(pipeline, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
    exit(-1);
  } else {
    ROS_INFO_STREAM("Stream is PAUSED.");
  }

  // Get the camera parameters file
  ros::param::param<std::string>("~/camera_parameters_file",camera_parameters_file,"../camera_parameters.txt"); // default path is to preserve backwards-compatibilitiy

  // We could probably do something with the camera name, check
  // errors or something, but at the moment, we don't care.
  if (camera_calibration_parsers::readCalibrationIni(camera_parameters_file, camera_name, camera_info)) {
    ROS_INFO_STREAM("Successfully read camera calibration from \""<<camera_parameters_file<<"\" for camera \""<<camera_name<<"\".  Rerun camera calibrator if it is incorrect.");
  }
  else {
    ROS_ERROR_STREAM("Camera parameters file \""<<camera_parameters_file<<"\" not found.  Use default file if no other is available.");
  }

  // Get TF Frame
  if(!ros::param::get("~/frame_id",camera_info.header.frame_id)){
    ROS_WARN("No camera frame_id set.");
  }

  // Pre-roll camera if needed
  int preroll;
  nh.param("brown/gscam/preroll", preroll, 0);
  if (preroll) {
    //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    //I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      std::cout << "Failed to PLAY." << std::endl;
      exit(-1);
    } else {
      std::cout << "stream is PLAYING." << std::endl;
    }

    gst_element_set_state(pipeline, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Failed to PAUSE.");
      exit(-1);
    } else {
      ROS_INFO("Stream is PAUSED.");
    }
  }

  // Create ROS camera interface
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher pub = it.advertiseCamera("camera/image_raw", 1);
  ros::ServiceServer set_camera_info = nh.advertiseService("camera/set_camera_info", setCameraInfo);

  ROS_INFO_STREAM("Processing...");

  //processVideo
  ROS_INFO("Starting stream...");
  GstStateChangeReturn gst_ret;
  
  gst_ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if(gst_ret == GST_STATE_CHANGE_FAILURE) {
    ROS_ERROR("Could not start stream!");
    exit(-1);
  }
  ROS_INFO("Started stream.");
  while(ros::ok()) {
    // This should block until a new frame is awake, this way, we'll run at the 
    // actual capture framerate of the device.
    ROS_DEBUG("Getting data...");
    GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink));
    ROS_DEBUG("Got data.");

    // Stop on end of stream
    if (!buf) {
      break;
    }

    GstPad* pad = gst_element_get_static_pad(sink, "sink");
    const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
    GstStructure *structure = gst_caps_get_structure(caps,0);
    gst_structure_get_int(structure,"width",&width);
    gst_structure_get_int(structure,"height",&height);

		// Complain if the returned buffer is smaller than we expect
		if (buf->size < unsigned(width * height * 3)) {
      ROS_WARN_STREAM( "GStreamer image buffer underflow: Expected frame to be "
          << (width * height * 3) << " bytes but got only "
          << (buf->size) << " bytes. (make sure frames are raw RGB encoded)");
		}

    // Construct Image message
    sensor_msgs::Image msg;

    // Header information
    camera_info.header.stamp = ros::Time::now();
    msg.header = camera_info.header;

    // Image data and metadata
    msg.width = width; 
    msg.height = height;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width*3;
    msg.data.resize(width*height*3);

    // Copy only the data we received
    std::copy(buf->data, (buf->data)+(buf->size), msg.data.begin());

    pub.publish(msg, camera_info);

    gst_buffer_unref(buf);

    ros::spinOnce();
  }

  // Clean up
  ROS_INFO("Stopping gstreamer pipeline...");
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
}
