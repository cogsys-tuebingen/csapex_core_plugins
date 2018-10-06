#ifndef REGISTER_CORE_PLUGINS_H
#define REGISTER_CORE_PLUGINS_H

/// HEADER
#include <csapex/core/core_plugin.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN RegisterCorePlugins : public CorePlugin
{
public:
    RegisterCorePlugins();

    void prepare(Settings&) override;
    void shutdown() override;
};

}  // namespace csapex

#endif  // REGISTER_CORE_PLUGINS_H
