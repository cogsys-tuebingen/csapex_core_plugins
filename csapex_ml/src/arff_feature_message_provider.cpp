/// HEADER
#include "arff_feature_message_provider.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>

/// SYSTEM
#include <cslibs_arff/arff_parser.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>

CSAPEX_REGISTER_CLASS(csapex::ARFFFeatureMessageProvider, csapex::MessageProvider)

using namespace csapex;

std::map<std::string, ARFFFeatureMessageProvider::ProviderConstructor> ARFFFeatureMessageProvider::plugins;

ARFFFeatureMessageProvider::ARFFFeatureMessageProvider() :
    next_feature_message_(-1)
{
    state.addParameter(param::ParameterFactory::declareBool("playing", true));
    state.addParameter(param::ParameterFactory::declareBool("loop", false));
    state.addParameter(param::ParameterFactory::declareRange("current_feature_message", 0, 1000, 0, 1));

    setType(connection_types::makeEmpty<connection_types::FeaturesMessage>());
}

void ARFFFeatureMessageProvider::load(const std::string& arff_file)
{
    cslibs_arff::ArffParser parser(arff_file);
    arff_ = parser.parse();

    features_messages_ = arff_->num_instances();

    csapex::param::Parameter::Ptr p = state.getParameter("current_feature_message");
    param::RangeParameter::Ptr range_p = std::dynamic_pointer_cast<param::RangeParameter>(p);

    if(range_p) {
        range_p->setMax(features_messages_ - 1);
    }
}

std::vector<std::string> ARFFFeatureMessageProvider::getExtensions() const
{
    return {".arff"};
}

bool ARFFFeatureMessageProvider::hasNext()
{
    if(!arff_) {
        return false;
    } else {
        if(state.readParameter<bool>("playback/resend")) {
            return true;
        }

        if(!state.readParameter<bool>("playing")) {
            // not resend and not playing
            return false;
        }

        if(state.readParameter<bool>("loop")) {
            return true;
        }

        int requested_message = state.readParameter<int>("current_feature_message");
        return next_feature_message_ < features_messages_ ||
                requested_message < features_messages_;
    }
}

connection_types::Message::Ptr ARFFFeatureMessageProvider::next(std::size_t slot)
{
    connection_types::FeaturesMessage::Ptr msg(new connection_types::FeaturesMessage);
    next(msg->value, msg->classification);
    std::cout << msg->value.front() << " " << msg->value.back() << std::endl;
    return msg;
}


void ARFFFeatureMessageProvider::next(std::vector<float> &value,
                                      int &classification)
{
    if(!arff_) {
        throw std::runtime_error("No arff file loaded!");
    }

    int requested_message = state.readParameter<int>("current_feature_message");
    if(state.readParameter<bool>("playing") || requested_message != next_feature_message_) {
        bool skip = next_feature_message_ != requested_message;

        if(next_feature_message_ >= features_messages_ && !skip) {
            value = last_data_;
            classification = last_classification_;
            setPlaying(false);
            return;
        }

        cslibs_arff::ArffInstance::Ptr arff_instance = arff_->get_instance(requested_message);

        last_data_.resize(arff_instance->size() - 1);
        for(std::size_t i = 0 ; i < last_data_.size() ; ++i) {
            last_data_[i] = (float) *(arff_instance->get(i));
        }

        std::stringstream ss;
        ss << (std::string) *(arff_instance->get(arff_instance->size() - 1));
        ss >> last_classification_;

        ++next_feature_message_;
        state["current_feature_message"] = next_feature_message_;

        if(next_feature_message_ == features_messages_) {
            bool loop = state.readParameter<bool>("loop");
            if(loop) {
                state["current_feature_message"] = 0;
            } else {
                setPlaying(false);
            }
        }
    }
    value  = last_data_;
    classification = last_classification_;

}

void ARFFFeatureMessageProvider::setPlaying(bool playing)
{
    state["playing"] = playing;
}

Memento::Ptr ARFFFeatureMessageProvider::getState() const
{
    GenericState::Ptr r(new GenericState(state));
    return r;
}

void ARFFFeatureMessageProvider::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<GenericState> m = std::dynamic_pointer_cast<GenericState> (memento);
    if(m) {
        state.setFrom(*m);
    }
}
