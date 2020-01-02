/// HEADER
#include "arff_feature_message_provider.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <cslibs_arff/arff_parser.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_CLASS(csapex::ARFFFeatureMessageProvider, csapex::MessageProvider)

using namespace csapex;
using namespace connection_types;

std::map<std::string, ARFFFeatureMessageProvider::ProviderConstructor> ARFFFeatureMessageProvider::plugins;

ARFFFeatureMessageProvider::ARFFFeatureMessageProvider() : sent_(false)
{
    setType(makeEmpty<GenericVectorMessage>());
}

void ARFFFeatureMessageProvider::load(const std::string& arff_file)
{
    cslibs_arff::ArffParser parser(arff_file);
    arff_ = parser.parse();
}

std::vector<std::string> ARFFFeatureMessageProvider::getExtensions() const
{
    return { ".arff" };
}

bool ARFFFeatureMessageProvider::hasNext()
{
    return !sent_ || state.readParameter<bool>("playback/resend");
}

connection_types::Message::Ptr ARFFFeatureMessageProvider::next(std::size_t slot)
{
    GenericVectorMessage::Ptr msg(makeEmpty<GenericVectorMessage>());
    std::shared_ptr<std::vector<FeaturesMessage>> msgs(new std::vector<FeaturesMessage>);
    const std::size_t instances = arff_->num_instances();
    if (instances == 0) {
        throw std::runtime_error("Tried to open empty file, nothing to playback here!");
    } else {
        const std::size_t step = arff_->get_instance(0)->size() - 1;
        std::cout << step << std::endl;

        for (std::size_t i = 0; i < instances; ++i) {
            cslibs_arff::ArffInstance::Ptr arff_instance = arff_->get_instance(i);
            FeaturesMessage f;
            const std::size_t instance_size = arff_instance->size() - 1;
            if (instance_size != step)
                throw std::runtime_error("All instances in ARFF file should have the same size!");

            f.value.resize(step);
            for (std::size_t i = 0; i < step; ++i) {
                f.value[i] = (float)*(arff_instance->get(i));
            }
            std::stringstream ss;
            ss << (std::string) * (arff_instance->get(arff_instance->size() - 1));
            ss >> f.classification;

            msgs->emplace_back(f);
        }
        msg->set(msgs);
        sent_ = true;
    }

    return msg;
}

GenericStatePtr ARFFFeatureMessageProvider::getState() const
{
    GenericState::Ptr r(new GenericState(state));
    return r;
}

void ARFFFeatureMessageProvider::setParameterState(GenericStatePtr memento)
{
    std::shared_ptr<GenericState> m = std::dynamic_pointer_cast<GenericState>(memento);
    if (m) {
        state.setFrom(*m);
    }
}
