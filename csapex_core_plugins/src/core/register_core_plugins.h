#ifndef REGISTER_CORE_PLUGINS_H
#define REGISTER_CORE_PLUGINS_H

/// HEADER
#include <csapex/core/core_plugin.h>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN RegisterCorePlugins : public CorePlugin
{
public:
    RegisterCorePlugins();

    void init(CsApexCore& core);
    void shutdown();
};

}

#endif // REGISTER_CORE_PLUGINS_H
