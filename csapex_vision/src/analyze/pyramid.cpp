/// HEADER
#include "pyramid.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/cv_pyramid_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::Pyramid, csapex::Node)

Pyramid::Pyramid() : out_levels_(8), out_level_idx_(0)
{
}

void Pyramid::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvPyramidMessage::Ptr out(new CvPyramidMessage(in->getEncoding()));

    cv::buildPyramid(in->value, out->value, out_levels_);

    if (msg::isConnected(out_level_)) {
        CvMatMessage::Ptr out_level(new CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));
        out_level->value = out->value.at(out_level_idx_).clone();
        msg::publish(out_level_, out_level);
    }

    msg::publish(out_pyr_, out);
}
void Pyramid::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    out_level_ = node_modifier.addOutput<CvMatMessage>("preview");
    out_pyr_ = node_modifier.addOutput<CvPyramidMessage>("pyramid");
}

void Pyramid::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("levels", 1, 10, out_levels_, 1), std::bind(&Pyramid::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("preview", 0, 9, out_level_idx_, 1), std::bind(&Pyramid::update, this));
}

void Pyramid::update()
{
    out_levels_ = readParameter<int>("levels");
    out_level_idx_ = readParameter<int>("preview");

    if (out_level_idx_ >= out_levels_) {
        out_level_idx_ = out_levels_ - 1;
        node_modifier_->setWarning("Not enough levels!");
    } else if (node_modifier_->isError()) {
        node_modifier_->setNoError();
    }
}
