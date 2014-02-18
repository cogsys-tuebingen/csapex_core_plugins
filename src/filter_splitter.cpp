#include "filter_splitter.h"

/// PROJECT
#include <csapex/view/box.h>
#include <csapex/command/meta.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>


/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Splitter, csapex::Node)

using namespace csapex;
using namespace connection_types;

Splitter::Splitter() :
    input_(NULL)
{
    state_.channel_count_ = 0;
}

void Splitter::setup()
{
    setSynchronizedInputs(true);

    /// add input
    input_ = addInput<CvMatMessage>("Image");

    updateOutputs();
}

void Splitter::process()
{
    CvMatMessage::Ptr m = input_->getMessage<CvMatMessage>();
    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    bool recompute = state_.channel_count_ != (int) channels.size();
    if(static_cast<unsigned>(m->getEncoding().size()) != channels.size()) {
        recompute = true;
    } else if(m->getEncoding().size() != state_.encoding_.size()) {
        recompute = true;
    } else {
        for(int i = 0, n = m->getEncoding().size(); i < n; ++i) {
            if(m->getEncoding()[i].name != state_.encoding_[i].name) {
                recompute = true;
                break;
            }
        }
    }
    if(recompute) {
        state_.encoding_ = m->getEncoding();
        state_.channel_count_ = channels.size();

        updateOutputs();
        Q_EMIT modelChanged();
        return;
    }


    for(unsigned i = 0 ; i < channels.size() ; i++) {
        Encoding e;
        e.push_back(state_.encoding_[i]);

        CvMatMessage::Ptr channel_out(new CvMatMessage(e));
        channel_out->value = channels[i];
        getOutput(i)->publish(channel_out);
    }
}

void Splitter::updateOutputs()
{
    unsigned n = countOutputs();

    if(state_.encoding_.size() > n) {
        for(unsigned i = n ; i < state_.encoding_.size() ; ++i) {
            addOutput<CvMatMessage>(state_.encoding_[i].name);
        }
    } else {
        bool del = true;
        for(int i = n-1 ; i >= (int) state_.encoding_.size(); --i) {
            if(getOutput(i)->isConnected()) {
                del = false;
            }

            if(del) {
                removeOutput(getOutput(i));
            } else {
                getOutput(i)->disable();;
            }
        }
    }

    for(int i = 0, n = state_.encoding_.size(); i < n; ++i) {
        getOutput(i)->setLabel(state_.encoding_[i].name);
        getOutput(i)->enable();
    }

}

/// MEMENTO ------------------------------------------------------------------------------------
Memento::Ptr Splitter::getState() const
{
    return boost::shared_ptr<State>(new State(state_));
}

void Splitter::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;

    while((int) state_.encoding_.size() < state_.channel_count_) {
        state_.encoding_.push_back(Channel("Channel", 0, 255));
    }

    Q_EMIT modelChanged();
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
