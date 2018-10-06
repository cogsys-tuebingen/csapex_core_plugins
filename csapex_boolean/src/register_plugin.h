#ifndef REGISTER_PLUGIN_H
#define REGISTER_PLUGIN_H

/// HEADER
#include <csapex/core/core_plugin.h>

namespace csapex
{
namespace boolean
{
class CSAPEX_EXPORT_PLUGIN RegisterPlugin : public CorePlugin
{
public:
    RegisterPlugin();

    void prepare(Settings&) override;
};

}  // namespace boolean

}  // namespace csapex

#endif  // REGISTER_PLUGIN_H
