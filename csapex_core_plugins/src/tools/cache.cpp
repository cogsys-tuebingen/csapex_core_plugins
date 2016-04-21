/// PROJECT
#include <csapex/model/tickable_node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/any_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>

/// SYSTEM
#include <deque>

namespace csapex
{

class Cache : public TickableNode
{
public:
    Cache()
        : playback_(false)
    {

    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        input = node_modifier.addOptionalInput<connection_types::AnyMessage>("multidimensional message");
        output = node_modifier.addOutput<connection_types::AnyMessage>("demultiplexed message");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(csapex::param::ParameterFactory::declareValue("buffer size", 128), [this](param::Parameter* p) {
            buffer_size_ = p->as<int>();
            while(msgs.size() > buffer_size_) {
                msgs.pop_front();
            }
        });
        params.addParameter(csapex::param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter*) {
            reset();
        });

        params.addParameter(csapex::param::ParameterFactory::declareValue("playback", false), playback_);

        auto playing = [this](){
            return playback_;
        };
        auto not_playing = [this](){
            return !playback_;
        };

        params.addConditionalParameter(csapex::param::ParameterFactory::declareTrigger("start"), not_playing, [this](param::Parameter*) {
            setParameter("playback", true);
        });
        params.addConditionalParameter(csapex::param::ParameterFactory::declareTrigger("stop"), playing, [this](param::Parameter*) {
            setParameter("playback", false);
        });

        params.addConditionalParameter(csapex::param::ParameterFactory::declareRange("frame", 0, 128, 0 , 1), playing);
        params.addConditionalParameter(csapex::param::ParameterFactory::declareRange("hz", 1.0, 400.0, 30.0, 0.1), playing, [this](param::Parameter* p){
            setTickFrequency(p->as<double>());
        });

        params.addConditionalParameter(csapex::param::ParameterFactory::declareValue("play", true), playing, playing_);
        params.addConditionalParameter(csapex::param::ParameterFactory::declareValue("loop", true), playing, loop_);
    }

    void reset()
    {
        msgs.clear();
        updateParams();
    }

    void updateParams()
    {
        param::RangeParameter::Ptr range = getParameter<param::RangeParameter>("frame");
        range->setMax<int>(msgs.size()-1);
    }

    void process() override
    {
        if(msg::hasMessage(input)) {
            auto m = msg::getMessage(input);

            msgs.push_back(m);

            updateParams();

            msg::publish(output, m);
        }
    }

    bool canTick() override
    {
        return playback_;
    }

    void tick() override
    {
        std::size_t frame = readParameter<int>("frame");
        apex_assert(frame < msgs.size());

        auto m = msgs.at(frame);
        msg::publish(output, m);

        if(playing_) {
            ++frame;
            if(frame < msgs.size()) {
                setParameter("frame", (int) frame);
            } else {
                if(loop_) {
                    setParameter("frame", 0);
                } else {
                    setParameter("playback", false);
                }
            }
        }
    }

private:
    Input* input;
    Output* output;

    std::size_t buffer_size_;
    std::deque<TokenConstPtr> msgs;

    bool playback_;
    bool playing_;
    bool loop_;
};

}

CSAPEX_REGISTER_CLASS(csapex::Cache, csapex::Node)
