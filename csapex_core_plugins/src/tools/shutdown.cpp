/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/end_of_program_message.h>
#include <csapex/utility/error_handling.h>

/// SYSTEM
#include <signal.h>

using namespace csapex::connection_types;


namespace csapex
{

class CSAPEX_EXPORT_PLUGIN Shutdown : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addOptionalInput<AnyMessage>("a");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void processMarker(const csapex::connection_types::MessageConstPtr &marker) override
    {
        if(std::dynamic_pointer_cast<EndOfProgramMessage const>(marker)) {
            csapex::error_handling::stop();
        }
    }

    void process() override {

    }

private:
    Input* in_;
};


}

CSAPEX_REGISTER_CLASS(csapex::Shutdown, csapex::Node)


