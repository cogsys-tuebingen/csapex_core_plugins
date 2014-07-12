/// HEADER
#include "pyramid.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/cv_pyramid_message.h>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Pyramid, csapex::Node)

Pyramid::Pyramid() :
    out_levels_(8),
    out_level_idx_(0)
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));
}

void Pyramid::process()
{
    CvMatMessage::Ptr       in = input_->getMessage<connection_types::CvMatMessage>();
    CvPyramidMessage::Ptr   out(new CvPyramidMessage(in->getEncoding()));

    cv::buildPyramid(in->value, out->value, out_levels_);

    if(out_level_->isConnected()) {
        CvMatMessage::Ptr out_level(new CvMatMessage(in->getEncoding()));
        out_level->value = out->value.at(out_level_idx_).clone();
        out_level_->publish(out_level);
    }

    out_pyr_->publish(out);
}
void Pyramid::setup()
{
    input_     = modifier_->addInput<CvMatMessage>("original");
    out_level_ = modifier_->addOutput<CvMatMessage>("preview");
    out_pyr_   = modifier_->addOutput<CvPyramidMessage>("pyramid");
}

void Pyramid::setupParameters()
{

    addParameter(param::ParameterFactory::declareRange("levels", 1, 10, out_levels_, 1),
                 boost::bind(&Pyramid::update, this));
    addParameter(param::ParameterFactory::declareRange("preview",0, 9, out_level_idx_, 1),
                 boost::bind(&Pyramid::update, this));
}

void Pyramid::update()
{
    out_levels_ = param<int>("levels");
    out_level_idx_ = param<int>("preview");

    if(out_level_idx_ >= out_levels_) {
        out_level_idx_ = out_levels_ - 1;
        setError(true, "Not enough levels!", EL_WARNING);
    } else if(isError()) {
        setError(false);
    }


}
