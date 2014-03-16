/// HEADER
#include "label_regions.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/flood.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::LabelRegions, csapex::Node)

LabelRegions::LabelRegions()
{
    addParameter(param::ParameterFactory::declareRange("edge value", 0, 255, 255, 1));
   // addParameter(param::ParameterFactory::declareRange("edge sigma", 0, 127, 0, 1));
}

void LabelRegions::process()
{
#warning "FIX ENCODING"
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new CvMatMessage(enc::unknown));

    uchar   edge   = param<int>("edge value");
    utils_cv::label(in->value, out->value, edge);

    output_->publish(out);
}

void LabelRegions::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    output_ = addOutput<CvMatMessage>("Preview");
}


