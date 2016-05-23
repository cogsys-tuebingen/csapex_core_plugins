/// HEADER
#include "assign_class.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>

CSAPEX_REGISTER_CLASS(csapex::AssignClass, csapex::Node)

using namespace csapex;
using namespace connection_types;

AssignClass::AssignClass()
{

}

void AssignClass::setup(NodeModifier &node_modifier)
{
    VectorProcessNode<FeaturesMessage>::setup(node_modifier);
    in_labels_   = node_modifier.addOptionalInput<GenericVectorMessage, int>("labels");
}

void AssignClass::setupParameters(Parameterizable &parameters)
{
    VectorProcessNode<FeaturesMessage>::setupParameters(parameters);
    parameters.addParameter(param::ParameterFactory::declareRange("label", 0, 255, 0, 1),
                            label_);
}

void AssignClass::processCollection(std::vector<FeaturesMessage *> &collection)
{
    if(msg::hasMessage(in_labels_)) {
        std::shared_ptr<std::vector<int> const> in_labels = msg::getMessage<GenericVectorMessage, int>(in_labels_);

        if(collection.size() != in_labels->size())
            throw std::runtime_error("Label count != FeatureMsg count!");

        for(std::size_t i = 0 ; i < collection.size() ; ++i) {
            FeaturesMessage &feature = *(collection.at(i));
            int label = (int) in_labels->at(i);
            feature.classification = label;
        }
    } else {
        std::cout << "label " << label_ << std::endl;
        std::cout << "msgs " << collection.size() << std::endl;
        for(FeaturesMessage *feature : collection) {
            std::cout << feature << std::endl;
            feature->classification = label_;
        }
    }
}
