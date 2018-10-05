/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <cslibs_laser_processing/common/yaml-io.hpp>
#include <cslibs_laser_processing/data/segment.h>

using namespace lib_laser_processing;
using namespace csapex::connection_types;

namespace csapex
{
class SegmentDistanceFilter : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, Segment>("Scan");
        out_ = modifier.addOutput<GenericVectorMessage, Segment>("Filtered Scan");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(csapex::param::factory::declareInterval("threshold", 0.0, 30.0, 0.4, 8.0, 0.01));
    }

    void process()
    {
        std::shared_ptr<std::vector<Segment> const> segments_in = msg::getMessage<GenericVectorMessage, Segment>(in_);

        std::pair<double, double> interval = readParameter<std::pair<double, double>>("threshold");
        double rmin = interval.first;
        double rmax = interval.second;

        std::vector<Segment> result;
        for (const Segment& segment : *segments_in) {
            for (const LaserBeam& beam : segment.rays) {
                if (beam.range() >= rmin && beam.range() <= rmax) {
                    result.push_back(segment);
                    break;
                }
            }
        }
        std::shared_ptr<std::vector<Segment>> segments_out(new std::vector<Segment>(result));

        msg::publish<GenericVectorMessage, Segment>(out_, segments_out);
    }

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::SegmentDistanceFilter, csapex::Node)
