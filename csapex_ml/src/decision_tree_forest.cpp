/// HEADER
#include "decision_tree_forest.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeForest, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

DecisionTreeForest::DecisionTreeForest()
{

}
void DecisionTreeForest::setup(NodeModifier &node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage::ConstPtr>("responses");

    reload_ = node_modifier.addSlot("Reload", [this](){load();});


}

void DecisionTreeForest::setupParameters(Parameterizable &parameters)
{

}

void DecisionTreeForest::process()
{

}

void DecisionTreeForest::load()
{

}
