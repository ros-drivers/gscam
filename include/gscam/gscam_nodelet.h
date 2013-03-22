#ifndef __GSCAM_GSCAM_NODELET_H
#define __GSCAM_GSCAM_NODELET_H

#include <nodelet/nodelet.h>

#include <gscam/gscam.h>

#include <boost/thread.hpp>

namespace gscam {
  class GSCamNodelet : public nodelet::Nodelet
  {
  public:
    GSCamNodelet();
    ~GSCamNodelet();

    virtual void onInit();

  private:
    boost::scoped_ptr<GSCam> gscam_driver_;
    boost::scoped_ptr<boost::thread> stream_thread_;
  };
}

#endif // infdef __GSCAM_GSCAM_NODELET_H
