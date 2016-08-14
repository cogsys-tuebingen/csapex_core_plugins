#include "canny.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Canny, csapex::Node)

Canny::Canny() :
    threshold_1_(3),
    threshold_2_(0.0),
    aperture_size_(255.0),
    L2_gradient_(false)
{
}

void Canny::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);

    if(!in->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->stamp_micro_seconds));

    cv::Mat edges;

    cv::Canny(in->value, edges, threshold_1_, threshold_2_, aperture_size_, L2_gradient_);

    out->value.create(in->value.size(), in->value.type());
    out->value.setTo(cv::Scalar::all(0));
    in->value.copyTo(out->value,edges);

    msg::publish(output_, out);
}

void Canny::setup(NodeModifier& node_modifier)
{
    CornerLineDetection::setup(node_modifier);
    update();
}

void Canny::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("aperture", 1, 7, 3, 2),
                 std::bind(&Canny::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("threshold 1", 0.0, 50000.0, 0.0, 1.0),
                 std::bind(&Canny::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("threshold 2", 0.0, 50000.0, 255.0, 1.0),
                 std::bind(&Canny::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("L2 gradient", false),
                 std::bind(&Canny::update, this));
}

void Canny::update()
{
    threshold_1_   = readParameter<double>("threshold 1");
    threshold_2_   = readParameter<double>("threshold 2");
    aperture_size_ = readParameter<int>("aperture");
    L2_gradient_   = readParameter<bool>("L2 gradient");
}
