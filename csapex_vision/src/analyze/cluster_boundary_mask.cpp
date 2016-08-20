/// HEADER
#include "cluster_boundary_mask.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <cslibs_vision/utils/cluster_boundaries.hpp>

CSAPEX_REGISTER_CLASS(csapex::ClusterBoundaryMask, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

ClusterBoundaryMask::ClusterBoundaryMask()
{

}

void ClusterBoundaryMask::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->stamp_micro_seconds));

    switch(in->value.type()) {
    case CV_8UC1:
        cslibs_vision::getClusterBoundaryMask<uint8_t>(in->value, out->value);
        break;
    case CV_8SC1:
        cslibs_vision::getClusterBoundaryMask<int8_t>(in->value, out->value);
        break;
    case CV_16UC1:
        cslibs_vision::getClusterBoundaryMask<uint16_t>(in->value, out->value);
        break;
    case CV_16SC1:
        cslibs_vision::getClusterBoundaryMask<int16_t>(in->value, out->value);
        break;
    case CV_32SC1:
        cslibs_vision::getClusterBoundaryMask<int32_t>(in->value, out->value);
        break;
    default:
        throw std::runtime_error("Only 1 channel integer matrices are supported!");
    }
    msg::publish(output_, out);
}

void ClusterBoundaryMask::setup(NodeModifier &node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("clusters");
    output_ = node_modifier.addOutput<CvMatMessage>("boundaries");
}

void ClusterBoundaryMask::setupParameters(Parameterizable &parameters)
{
}
