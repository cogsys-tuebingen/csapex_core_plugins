
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <cstdio>
#include <fcntl.h>
#include <future>
#include <signal.h>
#include <sys/wait.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class DirectExec : public Node
{
public:
    DirectExec()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareTrigger("execute"), [this](param::Parameter* param) {
            std::string command = cmd_;
            auto call = [this, command]() {
                const auto code = system(command.c_str());
                if (code != 0) {
                    aerr << "executing command " << command << " failed with code " << code << std::endl;
                }
            };
            if (async_)
                std::thread(call).detach();
            else
                call();
        });
        params.addParameter(param::factory::declareText("command", "echo 'Test'"), cmd_);
        params.addParameter(param::factory::declareBool("async", false), async_);
    }

    void process() override
    {
    }

private:
    std::string cmd_;
    bool async_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::DirectExec, csapex::Node)
