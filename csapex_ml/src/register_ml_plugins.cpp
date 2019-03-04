/// HEADER
#include <csapex_core_plugins/register_core_plugins.h>

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_state.h>
#include <csapex/model/tag.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/yaml_io.hpp>

using namespace csapex;

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN RegisterMlPlugins : public CorePlugin
{
public:
    RegisterMlPlugins() {}

    void prepare(Settings&) override
    {
        connection_types::GenericVectorMessage::registerType<connection_types::FeaturesMessage>();
    }
    void shutdown() override
    {
    }
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::RegisterMlPlugins, csapex::CorePlugin)