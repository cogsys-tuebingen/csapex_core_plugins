/// HEADER
#include "segmentation.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex/model/node_modifier.h>
#include <utils_laser_processing/data/segment.h>
#include <utils_laser_processing/common/yaml-io.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;
using namespace lib_laser_processing;

ScanSegmentation::ScanSegmentation()
{
}

void ScanSegmentation::process()
{
    /// WORK
    ScanMessage::ConstPtr scan_msg = msg::getMessage<ScanMessage>(input_);
    const Scan& scan = scan_msg->value;

    std::shared_ptr<std::vector<Segment> > segments_msg(new std::vector<Segment>);

    segmentation_->segmentation(scan, *segments_msg);

    for(Segment& segment : *segments_msg) {
        segment.frame_id = scan_msg->frame_id;
    }


    msg::publish<GenericVectorMessage, Segment>(output_segments_, segments_msg);
}

void ScanSegmentation::setup(NodeModifier& node_modifier)
{
    input_           = node_modifier.addInput<ScanMessage>("Scan");
    output_scan_     = node_modifier.addOutput<ScanMessage>("FilteredScan");
    output_segments_ = node_modifier.addOutput<GenericVectorMessage, Segment>("Segments");
}
