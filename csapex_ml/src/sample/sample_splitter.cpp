
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_ml/features_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class SampleSplitter : public Node
{
public:
    SampleSplitter()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<FeaturesMessage>("Features");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareValue("classification", 0));
    }

    void process() override
    {
        FeaturesMessage::ConstPtr features = msg::getMessage<FeaturesMessage>(in_);

        {
            std::size_t param_count = getPersistentParameters().size();
            if(param_count < features->value.size()){
                for(int i = param_count, n = features->value.size(); i < n; ++i) {
                    std::string name = std::string("entry_") + std::to_string(i);
                    param::Parameter::Ptr p = param::ParameterFactory::declareValue<double>(name, features->value.at(i));
                    addPersistentParameter(p);
                }
            }
        }

        std::vector<csapex::param::Parameter::Ptr> params = getPersistentParameters();
        for(int i = 0, n = features->value.size(); i < n; ++i) {
            param::ParameterPtr& p = params.at(i);
            p->set<double>(features->value.at(i));
        }

        setParameter("classification", features->classification);
    }

private:
    Input* in_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::SampleSplitter, csapex::Node)

