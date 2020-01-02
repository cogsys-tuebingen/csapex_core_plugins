/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <csapex_opencv/roi_message.h>

namespace csapex
{
using namespace connection_types;

class SetROIColorByClassification : public csapex::Node
{
public:
    SetROIColorByClassification()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        output_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    }
    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareValue("class.", -1), classification_);
        parameters.addParameter(param::factory::declareColorParameter("color", 0, 0, 0), color);
    }
    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> input_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(input_rois_);
        std::shared_ptr<std::vector<RoiMessage>> output_rois(new std::vector<RoiMessage>);

        for (const RoiMessage& r : *input_rois) {
            RoiMessage ro = r;
            if (ro.value.classification() == classification_) {
                ro.value.setColor(cv::Scalar(color[2], color[1], color[0]));
            }
            output_rois->emplace_back(ro);
        }
        msg::publish<GenericVectorMessage, RoiMessage>(output_rois_, output_rois);
    }

private:
    Input* input_rois_;

    Output* output_rois_;

    int classification_;
    std::vector<int> color;
};

}  // namespace csapex
CSAPEX_REGISTER_CLASS(csapex::SetROIColorByClassification, csapex::Node)
