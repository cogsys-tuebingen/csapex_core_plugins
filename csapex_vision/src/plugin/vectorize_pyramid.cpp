/// HEADER
#include "vectorize_pyramid.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/cv_pyramid_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::VectorizePyramid, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

VectorizePyramid::VectorizePyramid()
{
}

void VectorizePyramid::process()
{
    CvPyramidMessage::ConstPtr in = input_->getMessage<CvPyramidMessage>();
    std::shared_ptr<std::vector<CvMatMessage::Ptr> >
            out(new std::vector<CvMatMessage::Ptr>);

    Encoding enc = in->getEncoding();

    for(std::vector<cv::Mat>::const_iterator
        it = in->value.begin() ;
        it != in->value.end() ;
        ++it) {
        CvMatMessage::Ptr msg(new CvMatMessage(enc, in->stamp_micro_seconds));
        msg->value = it->clone();
        out->push_back(msg);
    }

    output_->publish<GenericVectorMessage, CvMatMessage::Ptr>(out);
}

void VectorizePyramid::setup()
{
    input_ = modifier_->addInput<CvPyramidMessage>("pyramid");
    output_ = modifier_->addOutput<GenericVectorMessage, CvMatMessage::Ptr>("vectorized");
}
