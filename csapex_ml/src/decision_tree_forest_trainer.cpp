/// HEADER
#include "decision_tree_forest_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeForestTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

DecisionTreeForestTrainer::DecisionTreeForestTrainer() :
    rand_vec_(2)
{

}

void DecisionTreeForestTrainer::setup(NodeModifier &modifier)
{

}

void DecisionTreeForestTrainer::setupParameters(Parameterizable &parameters)
{

}


bool DecisionTreeForestTrainer::processCollection(std::vector<connection_types::FeaturesMessage> &collection)
{

    return true;
}
