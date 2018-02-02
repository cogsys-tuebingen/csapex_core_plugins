
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <sys/wait.h>
#include <cstdio>
#include <signal.h>
#include <future>
#include <fcntl.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class DirectExec : public Node
{
public:
    DirectExec()
    {}

    void setup(csapex::NodeModifier& modifier) override
    {}

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareTrigger("execute"),
                            [this](param::Parameter* param)
                            {
                                std::string command = cmd_;
                                auto call = [command]() { system(command.c_str()); };
                                if (async_)
                                    std::thread(call).detach();
                                else
                                    call();
                            });
        params.addParameter(param::ParameterFactory::declareText("command", "echo 'Test'"), cmd_);
        params.addParameter(param::ParameterFactory::declareBool("async", false), async_);
    }

    void process() override
    {
    }

private:
    std::string cmd_;
    bool async_;
};
} // csapex


CSAPEX_REGISTER_CLASS(csapex::DirectExec, csapex::Node)

