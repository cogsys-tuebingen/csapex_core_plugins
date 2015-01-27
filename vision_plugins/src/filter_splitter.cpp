#include "filter_splitter.h"

/// PROJECT
#include <csapex/view/box.h>
#include <csapex/command/meta.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/assert.h>
#include <utils_param/parameter_factory.h>

CSAPEX_REGISTER_CLASS(csapex::Splitter, csapex::Node)

using namespace csapex;
using namespace connection_types;

Splitter::Splitter() :
    input_(nullptr)
{
    state_.channel_count_ = 0;    
}

Splitter::~Splitter()
{
}

void Splitter::setup()
{
    /// add input
    input_ = modifier_->addInput<CvMatMessage>("Image");

    updateOutputs();
}

void Splitter::setupParameters()
{
    addParameter(param::ParameterFactory::declareBool
                 ("enforce mono",
                  param::ParameterDescription("Enforce that the encoding is enc::mono"),
                  true));
}

void Splitter::process()
{
    CvMatMessage::ConstPtr m = input_->getMessage<CvMatMessage>();

    int esize = m->getEncoding().channelCount();
    if(esize != m->value.channels()) {
        std::stringstream error;
        error << "encoding size (" << m->getEncoding().channelCount() << ") != " << " image channels (" << m->value.channels() << ")";
        setError(true, error.str(), EL_WARNING);
    }

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    bool recompute = state_.channel_count_ != (int) channels.size();
    if(m->getEncoding().channelCount() != state_.encoding_.channelCount()) {
        recompute = true;
    } else {
        for(int i = 0, n = esize; i < n; ++i) {
            if(m->getEncoding().getChannel(i).name != state_.encoding_.getChannel(i).name) {
                recompute = true;
                break;
            }
        }
    }
    if(recompute) {
        state_.encoding_ = m->getEncoding();
        state_.channel_count_ = channels.size();

        input_->setLabel(m->getEncoding().toString());
        updateOutputs();
        triggerModelChanged();
        return;
    }

    bool enforce_mono = readParameter<bool>("enforce mono");

    std::vector<Output*> outputs = modifier_->getMessageOutputs();
    for(unsigned i = 0 ; i < channels.size() ; i++) {
        Encoding e;
        if(i < state_.encoding_.channelCount()) {
            if(enforce_mono) {
                e = enc::mono;
            } else {
                e.push_back(state_.encoding_.getChannel(i));
            }
        } else {
            e.push_back(enc::channel::unknown);
        }

        CvMatMessage::Ptr channel_out(new CvMatMessage(e, m->stamp_micro_seconds));
        channel_out->value = channels[i];
        outputs[i]->publish(channel_out);
    }
}

void Splitter::updateOutputs()
{
    std::vector<Output*> outputs = modifier_->getMessageOutputs();
    int n = outputs.size();

    if(state_.channel_count_ > n) {
        for(int i = n ; i < state_.channel_count_ ; ++i) {
            if(i < (int) state_.encoding_.channelCount()) {
                modifier_->addOutput<CvMatMessage>(state_.encoding_.getChannel(i).name);
            } else {
                modifier_->addOutput<CvMatMessage>("unknown");
            }
        }
    } else {
        bool del = true;
        for(int i = n-1 ; i >= (int) state_.channel_count_; --i) {
            Output* output = outputs[i];
            if(output->isConnected()) {
                del = false;
            }

            if(del) {
                modifier_->removeOutput(output->getUUID());
            } else {
                output->disable();
            }
        }
    }


    outputs = modifier_->getMessageOutputs();
    for(int i = 0, n = state_.channel_count_; i < n; ++i) {
        Output* output = outputs[i];
        if(i < (int) state_.encoding_.channelCount()) {
            output->setLabel(state_.encoding_.getChannel(i).name);
        } else {
            output->setLabel("unknown");
        }
        output->enable();
    }

}

/// MEMENTO ------------------------------------------------------------------------------------
//Memento::Ptr Splitter::getParameterState() const
//{
//    return std::shared_ptr<State>(new State(state_));
//}

//void Splitter::setParameterState(Memento::Ptr memento)
//{
//    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
//    apex_assert_hard(m.get());

//    state_ = *m;

////    while((int) state_.encoding_.size() < state_.channel_count_) {
////        state_.encoding_.push_back(Channel("Channel", 0, 255));
////    }

//    updateOutputs();
//    triggerModelChanged();
//}

/// MEMENTO
void Splitter::State::readYaml(const YAML::Node &node)
{
    channel_count_ = node["channel_count"].as<int>();
}


void Splitter::State::writeYaml(YAML::Node &out) const
{
    out["channel_count"] = channel_count_;
}
