/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/token.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/output_progress_parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <deque>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN Cache : public Node
{
public:
    Cache() : playback_(false)
    {
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        slot = node_modifier.addSlot<connection_types::AnyMessage>("multidimensional message", [this](const TokenPtr& token) {
            msgs.push_back(token->getTokenData());
            update();
        });
        output = node_modifier.addOutput<connection_types::AnyMessage>("demultiplexed message");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        csapex::param::Parameter::Ptr p = param::factory::declareOutputProgress("buffered messages");
        buffered_ = dynamic_cast<param::OutputProgressParameter*>(p.get());
        params.addParameter(p);

        params.addParameter(csapex::param::factory::declareValue("buffer size", 128), [this](param::Parameter* p) {
            buffer_size_ = p->as<int>();
            update();
        });

        params.addParameter(csapex::param::factory::declareTrigger("reset"), [this](param::Parameter*) { reset(); });

        params.addParameter(csapex::param::factory::declareValue("playback", false), playback_);

        auto playing = [this]() { return playback_; };
        auto not_playing = [this]() { return !playback_; };

        params.addConditionalParameter(csapex::param::factory::declareTrigger("start"), not_playing, [this](param::Parameter*) {
            setParameter("playback", true);
            yield();
        });
        params.addConditionalParameter(csapex::param::factory::declareTrigger("stop"), playing, [this](param::Parameter*) {
            setParameter("playback", false);
            yield();
        });

        params.addConditionalParameter(csapex::param::factory::declareRange("frame", 0, 128, 0, 1), playing);

        params.addConditionalParameter(csapex::param::factory::declareValue("play", true), playing, playing_);
        params.addConditionalParameter(csapex::param::factory::declareValue("loop", true), playing, loop_);
    }

    void reset()
    {
        msgs.clear();
        update();
    }

    void update()
    {
        while (msgs.size() > buffer_size_) {
            msgs.pop_front();
        }

        param::RangeParameter::Ptr range = getParameter<param::RangeParameter>("frame");
        range->setMax<int>(msgs.size() - 1);

        buffered_->setProgress(msgs.size(), buffer_size_);
    }

    bool canProcess() const override
    {
        return playback_;
    }

    void process() override
    {
        std::size_t frame = readParameter<int>("frame");
        apex_assert(frame < msgs.size());

        auto m = msgs.at(frame);
        msg::publish(output, m);

        if (playing_) {
            ++frame;
            if (frame < msgs.size()) {
                setParameter("frame", (int)frame);
            } else {
                if (loop_) {
                    setParameter("frame", 0);
                } else {
                    setParameter("playback", false);
                }
            }
        }
    }

private:
    Slot* slot;
    Output* output;

    std::size_t buffer_size_;
    param::OutputProgressParameter* buffered_;

    std::deque<TokenDataConstPtr> msgs;

    bool playback_;
    bool playing_;
    bool loop_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::Cache, csapex::Node)
