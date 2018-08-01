/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <cslibs_laser_processing/data/segment.h>
#include <cslibs_laser_processing/common/yaml-io.hpp>
#include <csapex/msg/generic_vector_message.hpp>

using namespace lib_laser_processing;
using namespace csapex::connection_types;


namespace csapex
{

class SegmentLabelFilter : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, Segment>("Scan");
        out_ = modifier.addOutput<GenericVectorMessage, Segment>("Filtered Scan");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(csapex::param::factory::declareInterval("labels", 0, 10, 1, 1, 1));
    }

    void process()
    {
        std::shared_ptr< std::vector<Segment> const > segments_in = msg::getMessage<GenericVectorMessage, Segment>(in_);

        std::pair<int, int> interval = readParameter<std::pair<int,int>>("labels");
        int lmin = interval.first;
        int lmax = interval.second;

        std::vector<Segment> result;
        for(const Segment& segment : *segments_in) {
            if(lmin <= segment.classification && segment.classification <= lmax) {
                result.push_back(segment);
            }
        }
        std::shared_ptr< std::vector<Segment> > segments_out (new std::vector<Segment>(result));

        msg::publish<GenericVectorMessage, Segment>(out_, segments_out);
    }

private:
    Input* in_;
    Output* out_;
};

}

CSAPEX_REGISTER_CLASS(csapex::SegmentLabelFilter, csapex::Node)
