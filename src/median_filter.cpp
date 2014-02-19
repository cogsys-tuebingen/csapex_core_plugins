#include "median_filter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::MedianFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MedianFilter::MedianFilter() :
    kernel_size_(3)
{
    addTag(Tag::get("Filter"));
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declareRange("kernel", 3, 31, kernel_size_, 2),
                 boost::bind(&MedianFilter::update, this));
}

void MedianFilter::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    //// TODO
    if(in->value.channels() > 4)
        throw std::runtime_error("To many channels!");

   // cv::medianBlur(in->value, out->value, kernel_size_);
    output_->publish(out);
}

void MedianFilter::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Unfiltered");
    output_ = addOutput<CvMatMessage>("Filtered");
    update();
}

void MedianFilter::update()
{
    kernel_size_ = param<int>("kernel");
}
