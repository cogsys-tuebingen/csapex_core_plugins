/// HEADER
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace connection_types;

FeaturesMessage::FeaturesMessage()
    : MessageTemplate<std::vector<float>, FeaturesMessage> ("Features")
{}

void FeaturesMessage::writeYaml(YAML::Emitter &yaml) const
{
    yaml << YAML::Flow << YAML::Key << "features" << YAML::Value;
    yaml << YAML::BeginSeq;
    std::vector<float>::const_iterator r = value.begin();
    for(; r != value.end(); ++r) {
        yaml << *r;
    }
    yaml << YAML::EndSeq;
}

void FeaturesMessage::readYaml(const YAML::Node &node)
{
    apex_assert_hard(node.Type() == YAML::NodeType::Sequence);
}
