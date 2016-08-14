#include "median_filter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(vision_plugins::MedianFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

MedianFilter::MedianFilter() :
    kernel_size_(3)
{
}

void MedianFilter::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    if(in->value.channels() > 4)
        throw std::runtime_error("To many channels!");

    cv::medianBlur(in->value, out->value, kernel_size_);
    msg::publish(output_, out);
}

void MedianFilter::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("filtered");
    update();
}

void MedianFilter::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("kernel", 3, 31, kernel_size_, 2),
                 std::bind(&MedianFilter::update, this));
}

void MedianFilter::update()
{
    kernel_size_ = readParameter<int>("kernel");
}
