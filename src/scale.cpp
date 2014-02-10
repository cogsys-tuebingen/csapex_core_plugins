/// HEADER
#include "scale.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::Scale, csapex::Node)

using namespace csapex::connection_types;
using namespace csapex;

Scale::Scale()
{
    addTag(Tag::get("Vision"));
    addParameter(param::ParameterFactory::declareRange("percent x", 1.0, 400.0, 100.0, 1.0),
                 boost::bind(&Scale::update, this));
    addParameter(param::ParameterFactory::declareRange("percent y", 1.0, 400.0, 100.0, 1.0),
                 boost::bind(&Scale::update, this));
    std::vector< std::pair<std::string, int> > modes;
    modes.push_back(std::make_pair("nearest", (int) CV_INTER_NN));
    modes.push_back(std::make_pair("linear", (int) CV_INTER_LINEAR));
    modes.push_back(std::make_pair("area", (int) CV_INTER_AREA));
    modes.push_back(std::make_pair("cubic", (int) CV_INTER_CUBIC));
    modes.push_back(std::make_pair("lanczos4", (int) CV_INTER_LANCZOS4));
    addParameter(param::ParameterFactory::declareParameterSet<int>("mode", modes), boost::bind(&Scale::update, this));

}

void Scale::allConnectorsArrived()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    if(!in->value.empty()) {
        cv::resize(in->value, out->value, cv::Size(), scales_[0] / 100.0, scales_[1] / 100.0, mode_);
    } else {
        throw std::runtime_error("Cannot scale empty images!");
    }

    output_->publish(out);
}

void Scale::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    output_ = addOutput<CvMatMessage>("Scale");
    update();
}

void Scale::update()
{
    scales_[0] = param<double>("percent x");
    scales_[1] = param<double>("percent y");
    mode_      = param<int>("mode");
}
