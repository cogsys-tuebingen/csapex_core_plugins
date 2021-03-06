/// HEADER
#include "label_regions.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <cslibs_vision/utils/flood.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::LabelRegions, csapex::Node)

LabelRegions::LabelRegions()
{
}

void LabelRegions::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);

    if (in->value.type() != CV_8UC1) {
        throw std::runtime_error("Edges should be mask with type of CV_8UC1!");
    }

    CvMatMessage::Ptr out(new CvMatMessage(enc::unknown, in->frame_id, in->stamp_micro_seconds));

    unsigned int threshold = readParameter<int>("area thresh");

    uchar edge = readParameter<int>("edge value");
    if (threshold > 0)
        cslibs_vision::label(in->value, out->value, edge, threshold);
    else
        cslibs_vision::label(in->value, out->value, edge);

    msg::publish(output_, out);
}

void LabelRegions::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("edges");
    output_ = node_modifier.addOutput<CvMatMessage>("labels");
}

void LabelRegions::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("edge value", 0, 255, 255, 1));
    parameters.addParameter(csapex::param::factory::declareRange("area thresh", 0, 1000, 0, 10));
}
