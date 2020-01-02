/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_evaluation/confusion_matrix_message.h>
#include <csapex_ml/features_message.h>
#include <csapex_opencv/roi_message.h>

namespace csapex
{
using namespace connection_types;

class EvaluateROIClassification : public csapex::Node
{
public:
    EvaluateROIClassification()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_gt_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("Ground Truth");
        input_prediction_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("Predicted");
        output_confusion_ = node_modifier.addOutput<ConfusionMatrixMessage>("Confusion");

        node_modifier.addSlot("Reset", std::bind(&EvaluateROIClassification::reset, this));
    }
    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareTrigger("reset"), std::bind(&EvaluateROIClassification::reset, this));
    }
    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> input_gt = msg::getMessage<GenericVectorMessage, RoiMessage>(input_gt_);
        std::shared_ptr<std::vector<RoiMessage> const> input_prediction = msg::getMessage<GenericVectorMessage, RoiMessage>(input_prediction_);

        if (input_gt->size() != input_prediction->size())
            throw std::runtime_error("Size of ground truth data must be the same as "
                                     "predicted vice versa!");

        const std::size_t size = input_gt->size();
        for (std::size_t i = 0; i < size; ++i) {
            const RoiMessage& gt = input_gt->at(i);
            const RoiMessage& prediction = input_prediction->at(i);
            confusion_.reportClassification(gt.value.classification(), prediction.value.classification());
        }

        ConfusionMatrixMessage::Ptr output_confusion(new ConfusionMatrixMessage);
        output_confusion->confusion = confusion_;
        msg::publish(output_confusion_, output_confusion);
    }

private:
    Input* input_gt_;
    Input* input_prediction_;

    Output* output_confusion_;

    ConfusionMatrix confusion_;

    void reset() override
    {
        confusion_.reset();
    }
};

}  // namespace csapex
CSAPEX_REGISTER_CLASS(csapex::EvaluateROIClassification, csapex::Node)
