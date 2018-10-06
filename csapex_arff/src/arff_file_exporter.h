#ifndef JANN_FORMAT_EXPORTER_H
#define JANN_FORMAT_EXPORTER_H

/// PROJECT
#include <csapex/model/token_data.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_core_plugins/collection_node.h>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <mutex>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN ARFFFileExporter : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    ARFFFileExporter();

    virtual void setup(NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    bool processCollection(std::vector<connection_types::FeaturesMessage>& collection) override;

private:
    std::string path_;
    std::string relation_name_;
};
}  // namespace csapex

#endif  // JANN_FORMAT_EXPORTER_H
