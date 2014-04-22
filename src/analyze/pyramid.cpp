/// HEADER
#include "pyramid.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_core_plugins/ros_message_conversion.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Pyramid, csapex::Node)

Pyramid::Pyramid() :
    amount_levels_(8),
    preview_level_(0)
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    addParameter(param::ParameterFactory::declareRange("levels", 1, 10, amount_levels_, 1),
                 boost::bind(&Pyramid::update, this));
    addParameter(param::ParameterFactory::declareRange("preview",0, 9, preview_level_, 1),
                 boost::bind(&Pyramid::update, this));
}

void Pyramid::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    boost::shared_ptr<std::vector<CvMatMessage::Ptr> > levels(new std::vector<CvMatMessage::Ptr>);

    std::vector<cv::Mat> pyramid;
    cv::buildPyramid(in->value, pyramid, amount_levels_);


    for(int i = 0 ; i < amount_levels_ ; ++i) {
        levels->push_back(CvMatMessage::Ptr(new CvMatMessage(in->getEncoding())));
        levels->at(i)->value = pyramid.at(i);
    }

    if(preview_->isConnected()) {
        CvMatMessage::Ptr preview(new CvMatMessage(in->getEncoding()));
        preview->value = pyramid.at(preview_level_);
        preview_->publish(preview);
    }

    levels_->publish<GenericVectorMessage, CvMatMessage::Ptr>(levels);
}
void Pyramid::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    preview_ = addOutput<CvMatMessage>("Preview");
    levels_  = addOutput<GenericVectorMessage, CvMatMessage::Ptr>("levels");
}

void Pyramid::update()
{
    amount_levels_ = param<int>("levels");
    preview_level_ = param<int>("preview");

    if(preview_level_ >= amount_levels_) {
        preview_level_ = amount_levels_ - 1;
        setError(true, "Not enough levels!", EL_WARNING);
    } else if(isError()) {
        setError(false);
    }


}
