/// HEADER
#include "histogram.h"

/// PROJECT
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/histogram.hpp>


using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::Equalize, csapex::Node)

Histogram::Histogram()
{
    Tag::createIfNotExists("Histogram");
    addTag(Tag::get("Histogram"));
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declareRange("bins", 1, 512, 255, 1));

}

void Histogram::process()
{
    int bins = param<int>("bins");
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::unknown);
    #warning "fix encoding"
     utils_cv::histogram()

}

void Histogram::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Input");
    output_ = addOutput<CvMatMessage>("Histograms");
}

