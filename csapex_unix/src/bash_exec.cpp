
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


class BashExec : public Node
{
public:
    BashExec()
        : active_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<std::string>("Input");
        out_ = modifier.addOutput<std::string>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareText("command", "cat | cut -b2-"), [this](param::Parameter* p){
            cmd_ = p->as<std::string>();
            reset();
        });
    }

    bool isAsynchronous() const override
    {
        return true;
    }

    void process(NodeModifier &node_modifier, Parameterizable &parameters, Continuation continuation) override
    {
        future_ = std::async(std::launch::async, [this, continuation]{
            execute(continuation);
        });
    }

private:
    void execute(Continuation continuation)
    {
        pid_t pid = 0;
        int pipe_in[2];
        int pipe_out[2];

        pipe(pipe_in);
        pipe(pipe_out);

        pid = fork();

        if (pid == 0)
        {
            // CHILD -> execute command
            close(pipe_out[0]);
            dup2(pipe_out[1], STDOUT_FILENO);

            close(pipe_in[1]);
            dup2(pipe_in[0], STDIN_FILENO);
            close(pipe_in[1]);

            std::string cmd = std::string("/bin/bash -l -c '") + cmd_ + "'";
            system(cmd.c_str());
            close(pipe_out[1]);
            std::quick_exit(0);


        } else {
            // PARENT -> observe child
            std::string value = msg::getValue<std::string>(in_);

            if (write(pipe_in[1], value.c_str(), value.size()) != (int) value.size()) {
                close(pipe_in[1]);
                throw std::runtime_error("failure writing to subprocess");
            }
            close(pipe_in[0]);
            close(pipe_in[1]);

            active_ = false;

            std::stringstream result;

            close(pipe_out[1]);
            active_ = true;
            while(active_)
            {
                char line[256];
                std::size_t nbytes = read(pipe_out[0], line, sizeof(line));
                if(nbytes == 0) {
                    active_ = false;
                }
                result << std::string(line, nbytes);;
            }


            close(pipe_out[0]);

            msg::publish(out_, result.str());

            continuation({});
        }
    }

    void reset() override
    {
        if(future_.valid()) {
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

} // csapex


CSAPEX_REGISTER_CLASS(csapex::BashExec, csapex::Node)

