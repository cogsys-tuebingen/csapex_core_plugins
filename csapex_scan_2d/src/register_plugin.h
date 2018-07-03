#ifndef REGISTER_PLUGIN_H
#define REGISTER_PLUGIN_H

/// HEADER
#include <csapex/core/core_plugin.h>

namespace csapex {

class RegisterScan2DPlugin : public CorePlugin
{
public:
    RegisterScan2DPlugin();

    void prepare(Settings&) override;
    void shutdown() override;
};

}

#endif // REGISTER_PLUGIN_H
