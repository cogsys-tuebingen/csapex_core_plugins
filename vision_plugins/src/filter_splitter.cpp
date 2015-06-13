#include "filter_splitter.h"

/// PROJECT
#include <csapex/view/box.h>
#include <csapex/command/meta.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/assert.h>
#include <utils_param/parameter_factory.h>
#include <csapex/core/serialization.h>

CSAPEX_REGISTER_CLASS(csapex::Splitter, csapex::Node)

using namespace csapex;
using namespace connection_types;

Splitter::Splitter() :
    input_(nullptr)
{
    channel_count_ = 0;
}

Splitter::~Splitter()
{
}

void Splitter::setup(NodeModifier& node_modifier)
{
    /// add input
    input_ = node_modifier.addInput<CvMatMessage>("Image");

    updateOutputs();
}

void Splitter::setupParameters(Parameterizable& parameters)
{
    addParameter(param::ParameterFactory::declareBool
                 ("enforce mono",
                  param::ParameterDescription("Enforce that the encoding is enc::mono"),
                  true));
}

void Splitter::process()
{
    CvMatMessage::ConstPtr m = msg::getMessage<CvMatMessage>(input_);

    int esize = m->getEncoding().channelCount();
    if(esize != m->value.channels()) {
        std::stringstream error;
        error << "encoding size (" << m->getEncoding().channelCount() << ") != " << " image channels (" << m->value.channels() << ")";
        modifier_->setWarning(error.str());
    }

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    bool recompute = channel_count_ != (int) channels.size();
    if(m->getEncoding().channelCount() != encoding_.channelCount()) {
        recompute = true;
    } else {
        for(int i = 0, n = esize; i < n; ++i) {
            if(m->getEncoding().getChannel(i).name != encoding_.getChannel(i).name) {
                recompute = true;
                break;
            }
        }
    }
    if(recompute) {
        encoding_ = m->getEncoding();
        channel_count_ = channels.size();

        msg::setLabel(input_, m->getEncoding().toString());
        updateOutputs();
        return;
    }

    bool enforce_mono = readParameter<bool>("enforce mono");

    std::vector<Output*> outputs = modifier_->getMessageOutputs();
    for(unsigned i = 0 ; i < channels.size() ; i++) {
        Encoding e;
        if(i < encoding_.channelCount()) {
            if(enforce_mono) {
                e = enc::mono;
            } else {
                e.push_back(encoding_.getChannel(i));
            }
        } else {
            e.push_back(enc::channel::unknown);
        }

        CvMatMessage::Ptr channel_out(new CvMatMessage(e, m->stamp_micro_seconds));
        channel_out->value = channels[i];
        msg::publish(outputs[i], channel_out);
    }
}

void Splitter::updateOutputs()
{
    std::vector<Output*> outputs = modifier_->getMessageOutputs();
    int n = outputs.size();

    if(channel_count_ > n) {
        for(int i = n ; i < channel_count_ ; ++i) {
            if(i < (int) encoding_.channelCount()) {
                modifier_->addOutput<CvMatMessage>(encoding_.getChannel(i).name);
            } else {
                modifier_->addOutput<CvMatMessage>("unknown");
            }
        }
    } else {
        bool del = true;
        for(int i = n-1 ; i >= (int) channel_count_; --i) {
            Output* output = outputs[i];
            if(msg::isConnected(output)) {
                del = false;
            }

            if(del) {
                modifier_->removeOutput(msg::getUUID(output));
            } else {
                msg::disable(output);
            }
        }
    }


    outputs = modifier_->getMessageOutputs();
    for(int i = 0, n = channel_count_; i < n; ++i) {
        Output* output = outputs[i];
        if(i < (int) encoding_.channelCount()) {
            msg::setLabel(output, encoding_.getChannel(i).name);
        } else {
            msg::setLabel(output, "unknown");
        }
        msg::enable(output);
    }

}

namespace csapex
{
class SplitterSerializer
{
public:
    static void serialize(const Splitter& splitter, YAML::Node& doc)
    {
        doc["channel_count"] = splitter.channel_count_;
    }

    static void deserialize(Splitter& splitter, const YAML::Node& doc)
    {
        if(doc["channel_count"].IsDefined()) {
            splitter.channel_count_ = doc["channel_count"].as<int>();
        }

        splitter.updateOutputs();
    }
};
}

CSAPEX_REGISTER_SERIALIZER(csapex::Splitter, SplitterSerializer)
