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

Splitter::~Splitter()
{
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

        /// we can't update connectors here, must be done in main thread?
        updateOutputs();
        Q_EMIT modelChanged();
        return;
    }


    for(unsigned i = 0 ; i < channels.size() ; i++) {
        Encoding e;
        if(i < state_.encoding_.size()) {
            e.push_back(state_.encoding_[i]);
        } else {
            e.push_back(Channel("unknown", 0, 1));
        }

        CvMatMessage::Ptr channel_out(new CvMatMessage(e));
        channel_out->value = channels[i];
        getOutput(i)->publish(channel_out);
    }
}

void Splitter::updateOutputs()
{
    int n = countOutputs();

    if(state_.channel_count_ > n) {
        for(int i = n ; i < state_.channel_count_ ; ++i) {
            if(i < (int) state_.encoding_.size()) {
                addOutput<CvMatMessage>(state_.encoding_[i].name);
            } else {
                addOutput<CvMatMessage>("unknown");
            }
        }
    } else {
        bool del = true;
        for(int i = n-1 ; i >= (int) state_.channel_count_; --i) {
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

    for(int i = 0, n = state_.channel_count_; i < n; ++i) {
        if(i < (int) state_.encoding_.size()) {
            getOutput(i)->setLabel(state_.encoding_[i].name);
        } else {
            getOutput(i)->setLabel("unknown");
        }
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
