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

CSAPEX_REGISTER_CLASS(csapex::Splitter, csapex::Node)

using namespace csapex;
using namespace connection_types;

Splitter::Splitter() :
    input_(NULL)
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

void Splitter::process()
{
    CvMatMessage::Ptr m = input_->getMessage<CvMatMessage>();

    int esize = m->getEncoding().size();
    if(esize != m->value.channels()) {
        std::stringstream error;
        error << "encoding size (" << m->getEncoding().size() << ") != " << " image channels (" << m->value.channels() << ")";
        setError(true, error.str(), EL_WARNING);
    }

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    bool recompute = state_.channel_count_ != (int) channels.size();
    if(m->getEncoding().size() != state_.encoding_.size()) {
        recompute = true;
    } else {
        for(int i = 0, n = esize; i < n; ++i) {
            if(m->getEncoding()[i].name != state_.encoding_[i].name) {
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


    std::vector<Output*> outputs = getMessageOutputs();
    for(unsigned i = 0 ; i < channels.size() ; i++) {
        Encoding e;
        if(i < state_.encoding_.size()) {
            e.push_back(state_.encoding_[i]);
        } else {
            e.push_back(enc::channel::unknown);
        }

        CvMatMessage::Ptr channel_out(new CvMatMessage(e));
        channel_out->value = channels[i];
        outputs[i]->publish(channel_out);
    }
}

void Splitter::updateOutputs()
{
    std::vector<Output*> outputs = getMessageOutputs();
    int n = outputs.size();

    if(state_.channel_count_ > n) {
        for(int i = n ; i < state_.channel_count_ ; ++i) {
            if(i < (int) state_.encoding_.size()) {
                modifier_->addOutput<CvMatMessage>(state_.encoding_[i].name);
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
                removeOutput(output->getUUID());
            } else {
                output->disable();
            }
        }
    }


    outputs = getMessageOutputs();
    for(int i = 0, n = state_.channel_count_; i < n; ++i) {
        Output* output = outputs[i];
        if(i < (int) state_.encoding_.size()) {
            output->setLabel(state_.encoding_[i].name);
        } else {
            output->setLabel("unknown");
        }
        output->enable();
    }

}

/// MEMENTO ------------------------------------------------------------------------------------
Memento::Ptr Splitter::getParameterState() const
{
    return boost::shared_ptr<State>(new State(state_));
}

void Splitter::setParameterState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state_ = *m;

//    while((int) state_.encoding_.size() < state_.channel_count_) {
//        state_.encoding_.push_back(Channel("Channel", 0, 255));
//    }

    updateOutputs();
    triggerModelChanged();
}

/// MEMENTO
void Splitter::State::readYaml(const YAML::Node &node)
{
    node["channel_count"] >> channel_count_;
}


void Splitter::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "channel_count" << YAML::Value << channel_count_;
}
