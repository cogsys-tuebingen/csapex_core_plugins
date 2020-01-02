/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class FeaturesMessageFilter : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_features_ = modifier.addMultiInput<GenericVectorMessage, FeaturesMessage>("Features");
        out_features_ = modifier.addOutput<AnyMessage>("Filtered");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareValue("expected classification", 1), exp_classification_);
    }

    void process() override
    {
        if (msg::isMessage<FeaturesMessage>(in_features_)) {
            FeaturesMessage::ConstPtr in_features = msg::getMessage<FeaturesMessage>(in_features_);
            apex_assert(in_features->type == FeaturesMessage::Type::CLASSIFICATION);
            if (in_features->classification == exp_classification_) {
                msg::publish(out_features_, in_features);
            }

        } else {
            std::shared_ptr<std::vector<FeaturesMessage> const> in_features = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_features_);

            std::shared_ptr<std::vector<FeaturesMessage>> out_features(new std::vector<FeaturesMessage>);
            const std::size_t size = in_features->size();
            for (std::size_t i = 0; i < size; ++i) {
                const FeaturesMessage& fm = in_features->at(i);
                apex_assert(fm.type == FeaturesMessage::Type::CLASSIFICATION);
                if (fm.classification == exp_classification_) {
                    out_features->emplace_back(fm);
                }
            }
            msg::publish<GenericVectorMessage, FeaturesMessage>(out_features_, out_features);
        }
    }

private:
    Input* in_features_;
    Output* out_features_;

    int exp_classification_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::FeaturesMessageFilter, csapex::Node)
