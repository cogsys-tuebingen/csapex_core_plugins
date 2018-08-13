/// HEADER
#include "static_roi.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/range_parameter.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>

CSAPEX_REGISTER_CLASS(csapex::StaticRoi, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

StaticRoi::StaticRoi()
{
}

void StaticRoi::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("h", 1, 4096, 100, 1));
    parameters.addParameter(csapex::param::factory::declareRange("w", 1, 4096, 100, 1));
    parameters.addParameter(csapex::param::factory::declareRange("x", 0, 4095, 0, 1));
    parameters.addParameter(csapex::param::factory::declareRange("y", 0, 4095, 0, 1));
}

void StaticRoi::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addOptionalInput<CvMatMessage>("matrix");
    out_ = node_modifier.addOutput<RoiMessage>("ROI");
}

void StaticRoi::process()
{
    RoiMessage::Ptr     roi(new RoiMessage);
    std::pair<int,int>  plane_size;

    if(msg::hasMessage(in_)) {
        CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(in_);

        plane_size.first  = in->value.cols;
        plane_size.second = in->value.rows;

    } else {
        plane_size.first  = 8192;
        plane_size.second = 8192;
    }

    param::RangeParameter::Ptr w = getParameter<param::RangeParameter>("w");
    param::RangeParameter::Ptr h = getParameter<param::RangeParameter>("h");
    param::RangeParameter::Ptr x = getParameter<param::RangeParameter>("x");
    param::RangeParameter::Ptr y = getParameter<param::RangeParameter>("y");

    if(w->max<int>() > plane_size.first) {
        w->setInterval(1, plane_size.first);
    }

    if(h->max<int>() > plane_size.second) {
        h->setInterval(1, plane_size.second);
    }

    int max_idx = plane_size.first  - w->as<int>() - 1;
    int max_idy = plane_size.second - h->as<int>() - 1;

    if(x->max<int>() != max_idx) {
        x->setInterval(0, max_idx);
    }
    if(y->max<int>() != max_idy) {
        y->setInterval(0, max_idy);
    }

    roi->value.setX(x->as<int>());
    roi->value.setY(y->as<int>());
    roi->value.setH(h->as<int>());
    roi->value.setW(w->as<int>());

    msg::publish(out_, roi);
}
