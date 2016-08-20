/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_ml/features_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;

class SetROIColorByClassification : public csapex::Node
{
public:
    SetROIColorByClassification();

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareValue("class.", -1),
                                classification_);
        parameters.addParameter(param::ParameterFactory::declareColorParameter("color", 0, 0, 0),
                                color);
    }
    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> input_rois =
                msg::getMessage<GenericVectorMessage, RoiMessage>(input_rois_);
        std::shared_ptr<std::vector<RoiMessage>> output_rois(new std::vector<RoiMessage>);

        for(const RoiMessage &r : *input_rois) {
            RoiMessage ro = r;
            if(ro.value.classification() == classification_) {
                ro.value.setColor(cv::Scalar(color[0], color[1], color[2]));
            }
            output_rois->emplace_back(ro);
        }
        msg::publish<GenericVectorMessage, RoiMessage>(output_rois_, output_rois);
    }

private:
    Input* input_rois_;
    Input* input_features_;

    Output* output_rois_;

    int                classification_;
    std::array<int, 3> color;
};

}
CSAPEX_REGISTER_CLASS(csapex::SetROIColorByClassification, csapex::Node)
