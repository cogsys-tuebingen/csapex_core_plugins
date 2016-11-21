#ifndef REGISTER_PLUGIN_H
#define REGISTER_PLUGIN_H

/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/model/observer.h>

namespace csapex {

class RegisterTransformPlugin : public CorePlugin, public Observer
{
public:
    RegisterTransformPlugin();

    void init(CsApexCore& core);
};

}

#endif // REGISTER_PLUGIN_H
