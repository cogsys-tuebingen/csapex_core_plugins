#include "mlp_cv.hpp"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/slot.h>

CSAPEX_REGISTER_CLASS(csapex::MLPCv, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MLPCv::MLPCv()
    : loaded_(false)
{
}

void MLPCv::setup(NodeModifier &node_modifier)
{
    input_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("Unclassified feature");
    output_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Classified features");

    reload_ = node_modifier.addSlot("Reload", std::bind(&MLPCv::loadMLP, this));
}

void MLPCv::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareFileInputPath("file", "mlp.yaml"),
                            std::bind(&MLPCv::loadMLP, this));
}

void MLPCv::loadMLP()
{
#if CV_MAJOR_VERSION == 2
    mlp_.load(readParameter<std::string>("file").c_str());
#elif CV_MAJOR_VERSION == 3
    mlp_->load(readParameter<std::string>("file"));
#endif
    loaded_ = true;
}

void MLPCv::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input =
            msg::getMessage<GenericVectorMessage, FeaturesMessage>(input_);

    std::shared_ptr<std::vector<FeaturesMessage>> output(new std::vector<FeaturesMessage>);

    if(loaded_) {
        std::size_t n = input->size();
        output->resize(n);
        for(std::size_t i = 0; i < n; ++i) {
           classify(input->at(i),output->at(i));
        }

    } else {
        *output = *input;
        node_modifier_->setWarning("cannot classfiy, no forest loaded");
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(output_, output);
}

void MLPCv::classify(const FeaturesMessage &input,
                     FeaturesMessage &output)
{
    output = input;

    cv::Mat feature(input.value);
    cv::Mat response;

#if CV_MAJOR_VERSION == 2
    mlp_.predict(feature, response);
#elif CV_MAJOR_VERSION == 3
    mlp_->predict(feature, response);
#endif

    cv::Point max;
    cv::minMaxLoc(response, nullptr, nullptr, nullptr, &max);

    output.classification = max.x;
}
