/// HEADER
#include "segment_labeler.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <utils_laser_processing/data/segment.h>
#include <utils_laser_processing/common/yaml-io.hpp>
#include <csapex_scan_2d/labeled_scan_message.h>

CSAPEX_REGISTER_CLASS(csapex::SegmentLabeler, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;


SegmentLabeler::SegmentLabeler()
{
}

void SegmentLabeler::setupParameters(Parameterizable& parameters)
{
}

void SegmentLabeler::setup(NodeModifier& node_modifier)
{
    in_segments_ = node_modifier.addInput<GenericVectorMessage, Segment>("Segments");
    in_labeled_scan_ = node_modifier.addInput<LabeledScanMessage>("Labeled Scan");

    out_ = node_modifier.addOutput<GenericVectorMessage, Segment>("Labeled Segments");
}

void SegmentLabeler::process()
{
    std::shared_ptr< std::vector<Segment> const > segments_in = msg::getMessage<GenericVectorMessage, Segment>(in_segments_);
    std::shared_ptr< std::vector<Segment> > segments_out (new std::vector<Segment>);

    LabeledScanMessage::ConstPtr scan = msg::getMessage<LabeledScanMessage>(in_labeled_scan_);
    const std::vector<int> &labels = scan->value.labels;

    segments_out->resize(segments_in->size());

    for(std::size_t s = 0, n = segments_in->size(); s < n; ++s) {
        Segment& segment = segments_out->at(s);
        segment = segments_in->at(s);

        if(segment.start_idx == -1) {
            throw std::runtime_error("Segments without start index are not supported!");
        }

        std::map<int,int> histogram;
        for(std::size_t ray_in_segment = 0, rays = segment.rays.size(); ray_in_segment < rays; ++ray_in_segment) {
            int classification = labels[segment.start_idx + ray_in_segment];
            ++histogram[classification];
        }

        int max = -1;
        int classification = -1;
        for(std::map<int,int>::const_iterator entry = histogram.begin(); entry != histogram.end(); ++entry) {
            if(entry->second > max) {
                max = entry->second;
                classification = entry->first;
            }
        }

        segment.classification = classification;
    }

    msg::publish<GenericVectorMessage, Segment>(out_, segments_out);
}

