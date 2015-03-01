/// HEADER
#include "vectorize_pyramid.h"

/// PROJECT
#include <csapex/msg/io.h>
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
    CvPyramidMessage::ConstPtr in = msg::getMessage<CvPyramidMessage>(input_);
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

    msg::publish<GenericVectorMessage, CvMatMessage::Ptr>(output_, out);
}

void VectorizePyramid::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvPyramidMessage>("pyramid");
    output_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage::Ptr>("vectorized");
}
