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
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Equalize, csapex::Node)

Equalize::Equalize()
{
    addTag(Tag::get("Histogram"));
    addTag(Tag::get("Filter"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));;
}

void Equalize::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));
    out->value    = in->value.clone();
    utils_cv::histogram::full_channel_equalize(out->value, out->value);
    output_->publish(out);
}

void Equalize::setup()
{
    input_ = addInput<CvMatMessage>("original");
    output_ = addOutput<CvMatMessage>("equalized");
}
