/// HEADER
#include "assign_class.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/msg/io.h>

CSAPEX_REGISTER_CLASS(csapex::AssignClass, csapex::Node)

using namespace csapex;
using namespace connection_types;

AssignClass::AssignClass()
{

}

void AssignClass::setup(NodeModifier &node_modifier)
{
    in_features_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    in_labels_   = node_modifier.addInput<GenericVectorMessage, int>("labels");
    out_         = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("labeled features");
}

void AssignClass::setupParameters(Parameterizable &parameters)
{

}

void AssignClass::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> in_features =
            msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_features_);
    std::shared_ptr<std::vector<int> const> in_labels =
            msg::getMessage<GenericVectorMessage, int>(in_labels_);
    std::shared_ptr<std::vector<FeaturesMessage> > out(new std::vector<FeaturesMessage>);

    if(in_features->size() != in_labels->size())
        throw std::runtime_error("Label count != FeatureMsg count!");

    for(unsigned int i = 0 ; i < in_features->size() ; ++i) {
        if(in_labels->at(i) > -1) {
            out->push_back(in_features->at(i));
            out->back().classification = in_labels->at(i);
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out);
}
