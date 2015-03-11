/// HEADER
#include "binomial_filter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/kernel.hpp>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::BinomialFilter, csapex::Node)

BinomialFilter::BinomialFilter()
{
}

void BinomialFilter::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    int kernel_size = readParameter<int>("kernel");
    if(kernel_size != kernel_size_) {
        utils_cv::buildBinomialKernel(kernel_, kernel_size);
        kernel_size_ = kernel_size;
    }

    cv::filter2D(in->value, out->value, -1, kernel_);
    msg::publish(output_, out);
}

void BinomialFilter::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("filtered");
}

void BinomialFilter::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("kernel", 3, 131, 3, 2));
}
