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

class ExchangeFeatureClassification : public csapex::Node
{
public:
    ExchangeFeatureClassification()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_features_  = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("Features");
        output_features_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Features");
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareValue("exchange", -1),
                                exchange_);
        parameters.addParameter(param::ParameterFactory::declareValue("by", 0),
                                by_);
    }
    virtual void process() override
    {
        std::shared_ptr<std::vector<FeaturesMessage> const> input_features =
                msg::getMessage<GenericVectorMessage, FeaturesMessage>(input_features_);
        std::shared_ptr<std::vector<FeaturesMessage>> output_features(new std::vector<FeaturesMessage>);

        for(const FeaturesMessage &fm : *input_features) {
            apex_assert(fm.type == FeaturesMessage::Type::CLASSIFICATION);
            if(fm.classification == exchange_) {
                output_features->emplace_back(fm);
                output_features->back().classification = by_;
            }
        }

        msg::publish<GenericVectorMessage, FeaturesMessage>(output_features_, output_features);
    }

private:
    Input* input_features_;

    Output* output_features_;

    int exchange_;
    int by_;

};

}
CSAPEX_REGISTER_CLASS(csapex::ExchangeFeatureClassification, csapex::Node)
