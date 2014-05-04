/// HEADER
#include "binomial_filter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/kernel.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::BinomialFilter, csapex::Node)

BinomialFilter::BinomialFilter()
{
    addTag(Tag::get("Filter"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    addParameter(param::ParameterFactory::declareRange("kernel", 3, 131, 3, 2));
}

void BinomialFilter::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    int kernel_size = param<int>("kernel");
    if(kernel_size != kernel_size_) {
        utils_cv::buildBinomialKernel(kernel_, kernel_size);
        kernel_size_ = kernel_size;
    }

    cv::filter2D(in->value, out->value, -1, kernel_);
    output_->publish(out);
}

void BinomialFilter::setup()
{
    input_ = addInput<CvMatMessage>("original");
    output_ = addOutput<CvMatMessage>("filtered");
}
