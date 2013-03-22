
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gscam/gscam_nodelet.h>

PLUGINLIB_DECLARE_CLASS(gscam, GSCamNodelet, gscam::GSCamNodelet, nodelet::Nodelet) 

namespace gscam {
  GSCamNodelet::GSCamNodelet() :
    nodelet::Nodelet(),
    gscam_driver_(NULL),
    stream_thread_(NULL)
  {
  }

  GSCamNodelet::~GSCamNodelet() 
  {
    stream_thread_->join();
  }

  void GSCamNodelet::onInit()
  {
    gscam_driver_.reset(new gscam::GSCam(this->getNodeHandle(), this->getPrivateNodeHandle()));
    stream_thread_.reset(new boost::thread(boost::bind(&GSCam::run, gscam_driver_.get())));
  }
}
