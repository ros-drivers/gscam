
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gscam/gscam_nodelet.h>

PLUGINLIB_DECLARE_CLASS(gscam, GSCamNodelet, gscam::GSCamNodelet, nodelet::Nodelet) 

namespace gscam {
  GSCamNodelet::GSCamNodelet() :
    nodelet::Nodelet(),
    gscam_driver(this->getNodeHandle(), this->getPrivateNodeHandle()),
    stream_thread_(NULL)
  {
  }

  GSCamNodelet::~GSCamNodelet() 
  {
    stream_thread_->join();
  }

  void GSCamNodelet::onInit()
  {
    stream_thread_.reset(new boost::thread(boost::bind(&GSCam::run, gscam_driver)));
  }
}
