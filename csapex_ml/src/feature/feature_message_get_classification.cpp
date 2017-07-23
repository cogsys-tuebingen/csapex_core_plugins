
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_ml/features_message.h>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class FeatureMessageGetClassification : public Node
{
public:
    FeatureMessageGetClassification()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, FeaturesMessage>("Input");
        out_ = modifier.addOutput<GenericVectorMessage, int>("Classification");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);

        std::shared_ptr<std::vector<int>> out(new std::vector<int>);
        for(const FeaturesMessage& f : *input){
            int classification = f.classification;
            out->emplace_back(classification);
        }
        msg::publish<GenericVectorMessage, int>(out_, out);
    }

private:
    Input* in_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::FeatureMessageGetClassification, csapex::Node)

