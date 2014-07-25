/// HEADER
#include "camera.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::Camera, csapex::Node)

using namespace csapex;

Camera::Camera()
    : current_dev_(-1)
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declare<int>("device", 0, 5, 0, 1), boost::bind(&Camera::update, this));

    w_ = 640;
    h_ = 480;

    addParameter(param::ParameterFactory::declare<int>("w", 640, 1280, w_, 1), boost::bind(&Camera::update, this));
    addParameter(param::ParameterFactory::declare<int>("h", 480, 800, h_, 1), boost::bind(&Camera::update, this));
}

void Camera::process()
{

}

void Camera::tick()
{
    if(cap_.isOpened()) {
        connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::bgr));
        cap_ >> msg->value;
        output_->publish(msg);
    }
}

void Camera::setup()
{
    output_ = modifier_->addOutput<connection_types::CvMatMessage>("Image");

    update();
}


void Camera::update()
{
    setError(false);


    int dev = readParameter<int>("device");
    int w = readParameter<int>("w");
    int h = readParameter<int>("h");


    if(dev == current_dev_ && w == w_ && h == h_) {
        // no change, no update
        return;
    }
    std::cerr << "update " << dev << " " << w << " " << h << std::endl;

    current_dev_= dev;
    w_ = w;
    h_ = h;

    if(cap_.isOpened()) {
        cap_.release();
    }

    if(!cap_.open(dev)) {
        cap_.release();
        throw std::runtime_error("cannot open camera with the given id");
    }

//    ainfo << "camera settings" << std::endl;
//    ainfo << cap_.get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cap_.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
//    cap_.set(CV_CAP_PROP_FRAME_WIDTH, w);
//    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, h);
//    ainfo << cap_.get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cap_.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
}
