
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
class SubprocessCall : public Node
{
public:
    SubprocessCall() : active_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<std::string>("Input");
        out_ = modifier.addOutput<std::string>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareText("command", "echo 'ok'"), [this](param::Parameter* p) {
            cmd_ = p->as<std::string>();
            reset();
        });
    }

    bool isAsynchronous() const override
    {
        return true;
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters, Continuation continuation) override
    {
        future_ = std::async(std::launch::async, [this, continuation] { execute(continuation); });
    }

private:
    void execute(Continuation continuation)
    {
        pid_t pid = 0;
        int pipe_in[2];
        int pipe_out[2];

        if(pipe(pipe_in) == -1) {
            aerr << "Cannot open pipe in" << std::endl;
            continuation({});
            return;
        }
        if(pipe(pipe_out) == -1) {
            aerr << "Cannot open pipe out" << std::endl;
            continuation({});
            return;
        }

        pid = fork();

        if (pid == 0) {
            // CHILD -> execute command
            close(pipe_out[0]);
            dup2(pipe_out[1], STDOUT_FILENO);

            close(pipe_in[1]);
            dup2(pipe_in[0], STDIN_FILENO);
            close(pipe_in[1]);

            std::string token;
            std::stringstream ss(cmd_);
            std::vector<std::string> tokens;
            while (ss >> token) {
                tokens.push_back(token);
            }

            if (tokens.empty()) {
                throw std::runtime_error("no command specified");
            }

            std::vector<char*> args;
            for (const std::string& token : tokens) {
                args.push_back(const_cast<char*>(token.data()));
            }
            args.push_back(nullptr);

            execvp(args[0], args.data());

        } else {
            // PARENT -> observe child
            std::string value = msg::getValue<std::string>(in_);

            if (write(pipe_in[1], value.c_str(), value.size()) != (int)value.size()) {
                close(pipe_in[1]);
                throw std::runtime_error("failure writing to subprocess");
            }
            close(pipe_in[0]);
            close(pipe_in[1]);

            active_ = false;

            std::stringstream result;

            close(pipe_out[1]);
            active_ = true;
            while (active_) {
                char line[256];
                std::size_t nbytes = read(pipe_out[0], line, sizeof(line));
                if (nbytes == 0) {
                    active_ = false;
                }
                result << std::string(line, nbytes);
                ;
            }

            close(pipe_out[0]);

            msg::publish(out_, result.str());

            continuation({});
        }
    }

    void reset() override
    {
        if (future_.valid()) {
            active_ = false;
            future_.wait();
        };
    }

private:
    Input* in_;
    Output* out_;

    std::string cmd_;
    bool active_;

    std::future<void> future_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::SubprocessCall, csapex::Node)
