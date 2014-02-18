/// HEADER
#include "equalize.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/histogram.hpp>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::Equalize, csapex::Node)

Equalize::Equalize()
{
    Tag::createIfNotExists("Histogram");
    addTag(Tag::get("Histogram"));
    addTag(Tag::get("Vision"));
}

void Equalize::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));
    out->value    = in->value.clone();
    utils_cv::full_channel_equalize(out->value, out->value);
    output_->publish(out);
}

void Equalize::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    output_ = addOutput<CvMatMessage>("Equalized");
}
