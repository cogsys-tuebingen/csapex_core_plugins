/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN RegisterMath : public CorePlugin
{
public:
    void prepare(Settings&) override
    {
        // do nothing here for now
        // TODO: register messages
    }
};

}

CSAPEX_REGISTER_CLASS(csapex::RegisterMath, csapex::CorePlugin)
