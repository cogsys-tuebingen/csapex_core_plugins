/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;

class SetROIClassification : public csapex::Node
{
public:
    SetROIClassification()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_rois_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        output_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareValue("set", -1),
                                set_);
    }
    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> inpurt_rois =
                msg::getMessage<GenericVectorMessage, RoiMessage>(input_rois_);
        std::shared_ptr<std::vector<RoiMessage>> output_rois(new std::vector<RoiMessage>);

        for(const RoiMessage &fm : *inpurt_rois) {
            output_rois->emplace_back(fm);
            output_rois->back().value.setClassification(set_);
        }

        msg::publish<GenericVectorMessage, RoiMessage>(output_rois_, output_rois);
    }

private:
    Input* input_rois_;
    Output* output_rois_;

    int set_;
};

}
CSAPEX_REGISTER_CLASS(csapex::SetROIClassification, csapex::Node)
