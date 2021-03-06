#include "split.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/serialization/node_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/utility/yaml.h>

CSAPEX_REGISTER_CLASS(csapex::Split, csapex::Node)

using namespace csapex;
using namespace connection_types;

Split::Split() : input_(nullptr)
{
    channel_count_ = 0;
}

Split::~Split()
{
}

void Split::setup(NodeModifier& node_modifier)
{
    /// add input
    input_ = node_modifier.addInput<CvMatMessage>("Image");

    updateOutputs();
}

void Split::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::factory::declareBool("enforce mono", csapex::param::ParameterDescription("Enforce that the encoding is enc::mono"), true));
}

void Split::process()
{
    CvMatMessage::ConstPtr m = msg::getMessage<CvMatMessage>(input_);

    int esize = m->getEncoding().channelCount();
    if (esize != m->value.channels()) {
        std::stringstream error;
        error << "encoding size (" << m->getEncoding().channelCount() << ") != "
              << " image channels (" << m->value.channels() << ")";
        node_modifier_->setWarning(error.str());
    }

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    bool recompute = channel_count_ != (int)channels.size();
    if (m->getEncoding().channelCount() != encoding_.channelCount()) {
        recompute = true;
    } else {
        for (int i = 0, n = esize; i < n; ++i) {
            if (m->getEncoding().getChannel(i).name != encoding_.getChannel(i).name) {
                recompute = true;
                break;
            }
        }
    }
    if (recompute) {
        encoding_ = m->getEncoding();
        channel_count_ = channels.size();

        msg::setLabel(input_, m->getEncoding().toString());
        updateOutputs();
        return;
    }

    bool enforce_mono = readParameter<bool>("enforce mono");

    std::vector<OutputPtr> outputs = node_modifier_->getMessageOutputs();
    for (unsigned i = 0; i < channels.size(); i++) {
        Encoding e;
        if (i < encoding_.channelCount()) {
            if (enforce_mono) {
                e = enc::mono;
            } else {
                e.push_back(encoding_.getChannel(i));
            }
        } else {
            e.push_back(enc::channel::unknown);
        }

        CvMatMessage::Ptr channel_out(new CvMatMessage(e, m->frame_id, m->stamp_micro_seconds));
        channel_out->value = channels[i];
        msg::publish(outputs[i].get(), channel_out);
    }
}

void Split::updateOutputs()
{
    std::vector<OutputPtr> outputs = node_modifier_->getMessageOutputs();
    int n = outputs.size();

    if (channel_count_ > n) {
        for (int i = n; i < channel_count_; ++i) {
            if (i < (int)encoding_.channelCount()) {
                node_modifier_->addOutput<CvMatMessage>(encoding_.getChannel(i).name);
            } else {
                node_modifier_->addOutput<CvMatMessage>("unknown");
            }
        }
    } else {
        bool del = true;
        for (int i = n - 1; i >= (int)channel_count_; --i) {
            Output* output = outputs[i].get();
            if (msg::isConnected(output)) {
                del = false;
            }

            if (del) {
                node_modifier_->removeOutput(msg::getUUID(output));
            } else {
                msg::disable(output);
            }
        }
    }

    outputs = node_modifier_->getMessageOutputs();
    for (int i = 0, n = channel_count_; i < n; ++i) {
        Output* output = outputs[i].get();
        if (i < (int)encoding_.channelCount()) {
            msg::setLabel(output, encoding_.getChannel(i).name);
        } else {
            msg::setLabel(output, "unknown");
        }
        msg::enable(output);
    }
}

namespace csapex
{
class SplitSerializer
{
public:
    static void serialize(const Split& splitter, YAML::Node& doc)
    {
        doc["channel_count"] = splitter.channel_count_;
    }

    static void deserialize(Split& splitter, const YAML::Node& doc)
    {
        if (doc["channel_count"].IsDefined()) {
            splitter.channel_count_ = doc["channel_count"].as<int>();
        }

        splitter.updateOutputs();
    }
};
}  // namespace csapex

CSAPEX_REGISTER_SERIALIZER(csapex::Split, SplitSerializer)
