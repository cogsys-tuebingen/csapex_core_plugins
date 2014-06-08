/// HEADER
#include "absolute_difference.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::AbsoluteDifference, csapex::Node)

AbsoluteDifference::AbsoluteDifference()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));
}

void AbsoluteDifference::process()
{
    CvMatMessage::Ptr mat_1  = mat_1_in_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr mat_2  = mat_2_in_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr result(new CvMatMessage(mat_1->getEncoding()));

#warning FIX ENCODING CHECK
    if(mat_1->value.type() != mat_2->value.type())
        throw std::runtime_error(
                "Cannot build abs. difference of different mat types!");
    if(mat_1->value.size() != mat_2->value.size())
        throw std::runtime_error(
                "Cannot build abs. difference with diffenrent mat sizes!");

    cv::absdiff(mat_1->value, mat_2->value, result->value);
    result_->publish(result);
}

void AbsoluteDifference::setup()
{
    mat_1_in_ =  modifier_->addInput<CvMatMessage>("matrix 1");
    mat_2_in_ =  modifier_->addInput<CvMatMessage>("matrix 2");
    result_   = modifier_->addOutput<CvMatMessage>("abs. difference");
}


