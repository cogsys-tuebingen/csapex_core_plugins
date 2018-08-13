/// HEADER
#include "camera.h"

/// COMPONENT
#include <csapex_opencv/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::Camera, csapex::Node)

using namespace csapex;

Camera::Camera()
    : current_dev_(-1)
{
}

void Camera::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange<int>("device", 0, 5, 0, 1), std::bind(&Camera::update, this));

    w_ = 640;
    h_ = 480;

    parameters.addParameter(csapex::param::factory::declareRange<int>("w", 640, 1280, w_, 1), std::bind(&Camera::update, this));
    parameters.addParameter(csapex::param::factory::declareRange<int>("h", 480, 800, h_, 1), std::bind(&Camera::update, this));
}

void Camera::process()
{
    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::bgr, "camera", 0));
    cap_ >> msg->value;
    if(msg->value.channels() == 1)
        msg->setEncoding(enc::mono);

    msg::publish(output_, msg);
}

bool Camera::canProcess() const
{
    return cap_.isOpened();
}

void Camera::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<connection_types::CvMatMessage>("Image");

    update();
}


void Camera::update()
{
    node_modifier_->setNoError();


    int dev = readParameter<int>("device");
    int w = readParameter<int>("w");
    int h = readParameter<int>("h");


    if(dev == current_dev_ && w == w_ && h == h_) {
        // no change, no update
        return;
    }
    ainfo << "update " << dev << " " << w << " " << h << std::endl;

    current_dev_= dev;
    w_ = w;
    h_ = h;

    if(cap_.isOpened()) {
        ainfo << "release" << std::endl;
        cap_.release();
    }

    if(!cap_.open(dev)) {
        cap_.release();
        throw std::runtime_error("cannot open camera with the given id");
    } else {
        ainfo << "opened camera " << dev << std::endl;
    }

    //    ainfo << "camera settings" << std::endl;
    //    ainfo << cap_.get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cap_.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    //    cap_.set(CV_CAP_PROP_FRAME_WIDTH, w);
    //    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, h);
    //    ainfo << cap_.get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cap_.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
}
