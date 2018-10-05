
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <map>
#include <signal.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class UnixSignalEmitter : public Node
{
public:
    UnixSignalEmitter()
      : signal_list({ { SIGHUP, "SIGHUP    - Hangup" },
                      { SIGINT, "SIGINT    - Interrupt" },
                      { SIGQUIT, "SIGQUIT   - Quit" },
                      { SIGILL, "SIGILL    - Illegal instruction" },
                      { SIGTRAP, "SIGTRAP   - Trace trap" },
                      { SIGABRT, "SIGABRT   - Abort" },
                      { SIGIOT, "SIGIOT    - IOT trap" },
                      { SIGBUS, "SIGBUS    - BUS error" },
                      { SIGFPE, "SIGFPE    - Floating-point exception" },
                      { SIGKILL, "SIGKILL   - Kill, unblockable" },
                      { SIGUSR1, "SIGUSR1   - User-defined signal 1" },
                      { SIGSEGV, "SIGSEGV   - Segmentation violation" },
                      { SIGUSR2, "SIGUSR2   - User-defined signal 2" },
                      { SIGPIPE, "SIGPIPE   - Broken pipe" },
                      { SIGALRM, "SIGALRM   - Alarm clock" },
                      { SIGTERM, "SIGTERM   - Termination" },
                      { SIGSTKFLT, "SIGSTKFLT - Stack fault" },
                      { SIGCHLD, "SIGCHLD   - Child status has changed" },
                      { SIGCONT, "SIGCONT   - Continue" },
                      { SIGSTOP, "SIGSTOP   - Stop, unblockable" },
                      { SIGTSTP, "SIGTSTP   - Keyboard stop" },
                      { SIGTTIN, "SIGTTIN   - Background read from tty" },
                      { SIGTTOU, "SIGTTOU   - Background write to tty" },
                      { SIGURG, "SIGURG    - Urgent condition on socket" },
                      { SIGXCPU, "SIGXCPU   - CPU limit exceeded" },
                      { SIGXFSZ, "SIGXFSZ   - File size limit exceeded" },
                      { SIGVTALRM, "SIGVTALRM - Virtual alarm clock" },
                      { SIGPROF, "SIGPROF   - Profiling alarm clock" },
                      { SIGWINCH, "SIGWINCH  - Window size change (4.3 BSD, Sun)" },
                      { SIGPOLL, "SIGPOLL   - Pollable event occurred (System V)" },
                      { SIGIO, "SIGIO     - I/O now possible" },
                      { SIGPWR, "SIGPWR    - Power failure restart (System V)" },
                      { SIGSYS, "SIGSYS    - Bad system call" } })
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        modifier.addInput<csapex::connection_types::AnyMessage>("trigger");

        modifier.addSlot("trigger", [this]() { trigger(); });
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        std::map<std::string, int> signal;
        for (const auto& pair : signal_list) {
            signal.insert({ pair.second, pair.first });
        }

        params.addParameter(param::factory::declareParameterSet("signal", signal, SIGUSR1), signal_);
    }

    void process() override
    {
        trigger();
    }

    void trigger()
    {
        ainfo << "raise " << signal_list.at(signal_) << std::endl;
        raise(signal_);
    }

private:
    std::map<int, const char*> signal_list;
    int signal_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::UnixSignalEmitter, csapex::Node)
