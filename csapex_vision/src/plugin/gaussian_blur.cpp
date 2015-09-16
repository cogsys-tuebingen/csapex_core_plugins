/// HEADER
#include "gaussian_blur.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::GaussianBlur, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

GaussianBlur::GaussianBlur() :
    kernel_(1),
    sigma_x_(0.1),
    sigma_y_(0.0)
{
}

void GaussianBlur::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("kernel", 1, 255, kernel_, 2),
                 std::bind(&GaussianBlur::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("sigma x", 0.1, 128.0, sigma_x_, 0.1),
                 std::bind(&GaussianBlur::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("sigma y", 0.0, 128.0, sigma_y_, 0.1),
                 std::bind(&GaussianBlur::update, this));
}

void GaussianBlur::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    cv::GaussianBlur(in->value, out->value, cv::Size(kernel_, kernel_), sigma_x_, sigma_y_);
    msg::publish(output_, out);
}

void GaussianBlur::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Unblurred");
    output_ = node_modifier.addOutput<CvMatMessage>("Blurred");

    update();
}

void GaussianBlur::update()
{
    kernel_  = readParameter<int>("kernel");
    sigma_x_ = readParameter<double>("sigma x");
    sigma_y_ = readParameter<double>("sigma y");
}
