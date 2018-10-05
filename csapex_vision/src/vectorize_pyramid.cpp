/// HEADER
#include "vectorize_pyramid.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/cv_pyramid_message.h>

CSAPEX_REGISTER_CLASS(csapex::VectorizePyramid, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

VectorizePyramid::VectorizePyramid()
{
}

void VectorizePyramid::process()
{
    CvPyramidMessage::ConstPtr in = msg::getMessage<CvPyramidMessage>(input_);
    std::shared_ptr<std::vector<CvMatMessage::ConstPtr>> out(new std::vector<CvMatMessage::ConstPtr>);

    Encoding enc = in->getEncoding();

    for (std::vector<cv::Mat>::const_iterator it = in->value.begin(); it != in->value.end(); ++it) {
        CvMatMessage::Ptr msg(new CvMatMessage(enc, in->frame_id, in->stamp_micro_seconds));
        msg->value = it->clone();
        out->push_back(msg);
    }

    msg::publish<GenericVectorMessage, CvMatMessage::ConstPtr>(output_, out);
}

void VectorizePyramid::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvPyramidMessage>("pyramid");
    output_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage::ConstPtr>("vectorized");
}
