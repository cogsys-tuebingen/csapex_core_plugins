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

class AssignROIClassification : public csapex::Node
{
public:
    AssignROIClassification()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        input_features_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("Features");
        output_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    }
    void setupParameters(Parameterizable& parameters) override
    {
    }
    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> input_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(input_rois_);
        std::shared_ptr<std::vector<FeaturesMessage> const> input_features = msg::getMessage<GenericVectorMessage, FeaturesMessage>(input_features_);
        std::shared_ptr<std::vector<RoiMessage>> output_rois(new std::vector<RoiMessage>);

        if (input_rois->size() != input_features->size())
            throw std::runtime_error("Amount of ROIs and Features must be the same!");

        std::size_t size = input_rois->size();
        for (std::size_t i = 0; i < size; ++i) {
            FeaturesMessage fm = input_features->at(i);
            apex_assert(fm.type == FeaturesMessage::Type::CLASSIFICATION);
            RoiMessage roi = input_rois->at(i);
            roi.value.setClassification(fm.classification);
            if (roi.value.classification() > 0)
                roi.value.setColor(cv::Scalar(0, 255, 0));
            else
                roi.value.setColor(cv::Scalar(0, 0, 255));
            output_rois->push_back(roi);
        }
        msg::publish<GenericVectorMessage, RoiMessage>(output_rois_, output_rois);
    }

private:
    Input* input_rois_;
    Input* input_features_;

    Output* output_rois_;
};

}  // namespace csapex
CSAPEX_REGISTER_CLASS(csapex::AssignROIClassification, csapex::Node)
