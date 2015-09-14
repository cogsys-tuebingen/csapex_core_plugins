 /// COMPONENT
#include <csapex/model/node.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_laser_processing/data/segment.h>
#include <utils_laser_processing/common/yaml-io.hpp>
#include <csapex_core_plugins/vector_message.h>

using namespace lib_laser_processing;
using namespace csapex::connection_types;


namespace csapex
{

class SegmentMinCountFilter : public Node
{
public:
    SegmentMinCountFilter() :
        max_size_(200)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, Segment>("Scan");
        out_ = modifier.addOutput<GenericVectorMessage, Segment>("Filtered Scan");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("threshold", 2, (int) max_size_, 5, 1));
    }

    void process()
    {
        std::shared_ptr< std::vector<Segment> const > segments_in = msg::getMessage<GenericVectorMessage, Segment>(in_);
        std::shared_ptr< std::vector<Segment> > segments_out (new std::vector<Segment>());

        param::RangeParameter::Ptr p = std::dynamic_pointer_cast<param::RangeParameter>(getParameter("threshold"));
        size_t threshold = (size_t) readParameter<int>("threshold");

        for(auto it = segments_in->begin() ;
                 it != segments_in->end() ;
                 ++it) {
            if(max_size_ < it->rays.size()) {
                max_size_ = it->rays.size();
                p->setInterval<int>(2, max_size_);
            }
            if(it->rays.size() >= threshold) {
                segments_out->push_back(*it);
            }

        }

        msg::publish<GenericVectorMessage, Segment>(out_, segments_out);
    }

private:
    size_t max_size_;

    Input* in_;
    Output* out_;
};

}

CSAPEX_REGISTER_CLASS(csapex::SegmentMinCountFilter, csapex::Node)
