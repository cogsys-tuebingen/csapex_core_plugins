/// HEADER
#include "segment_labeler.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_param/parameter_factory.h>
#include <utils_laser_processing/data/segment.h>
#include <utils_laser_processing/common/yaml-io.hpp>
#include <csapex_scan_2d/labeled_scan_message.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::SegmentLabeler, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;


SegmentLabeler::SegmentLabeler()
{
}

void SegmentLabeler::setupParameters()
{
}

void SegmentLabeler::setup()
{
    in_segments_ = modifier_->addInput<GenericVectorMessage, Segment>("Segments");
    in_labeled_scan_ = modifier_->addInput<LabeledScanMessage>("Labeled Scan");

    out_ = modifier_->addOutput<GenericVectorMessage, Segment>("Labeled Segments");
}

void SegmentLabeler::process()
{
    boost::shared_ptr< std::vector<Segment> const > segments_in = in_segments_->getMessage<GenericVectorMessage, Segment>();
    boost::shared_ptr< std::vector<Segment> > segments_out (new std::vector<Segment>);

    LabeledScanMessage::Ptr scan = in_labeled_scan_->getMessage<LabeledScanMessage>();
    std::vector<int>::const_iterator labeled_ray = scan->value.labels.begin();

    segments_out->resize(segments_in->size());

    for(std::size_t s = 0, n = segments_in->size(); s < n; ++s) {
        Segment& segment = segments_out->at(s);
        segment = segments_in->at(s);

        std::map<int,int> histogram;
        for(std::size_t ray_in_segment = 0, rays = segment.rays.size(); ray_in_segment < rays; ++ray_in_segment) {
            int classification = *labeled_ray;
            ++histogram[classification];

            ++labeled_ray;
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

    out_->publish<GenericVectorMessage, Segment>(segments_out);

}

