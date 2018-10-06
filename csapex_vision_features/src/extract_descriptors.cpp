/// HEADER
#include "extract_descriptors.h"

/// COMPONENT
#include <csapex_vision_features/descriptor_message.h>
#include <csapex_vision_features/extractor_factory.h>
#include <csapex_vision_features/extractor_manager.h>
#include <csapex_vision_features/keypoint_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/param/set_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <cslibs_vision/utils/extractor.h>

/// SYSTEM
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::ExtractDescriptors, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractDescriptors::ExtractDescriptors() : refresh_(true)
{
}

void ExtractDescriptors::setupParameters(Parameterizable& parameters)
{
    ExtractorManager& manager = ExtractorManager::instance();
    std::vector<std::string> methods;

    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    for (Pair fc : manager.descriptorExtractors()) {
        std::string key = fc.second.getType();
        methods.push_back(key);
    }

    csapex::param::Parameter::Ptr method = csapex::param::ParameterFactory::declareParameterStringSet("method", methods);
    parameters.addParameter(method, std::bind(&ExtractDescriptors::update, this));

    for (Pair fc : manager.descriptorExtractors()) {
        std::string key = fc.second.getType();
        std::function<bool()> condition = [method, key]() { return method->as<std::string>() == key; };

        for (csapex::param::Parameter::Ptr param : manager.featureDescriptorParameters(key)) {
            csapex::param::Parameter::Ptr param_clone = csapex::param::ParameterFactory::clone(param);
            parameters.addConditionalParameter(param_clone, condition, std::bind(&ExtractDescriptors::update, this));
        }
    }
}

void ExtractDescriptors::setup(NodeModifier& node_modifier)
{
    in_img = node_modifier.addInput<CvMatMessage>("Image");
    in_key = node_modifier.addInput<KeypointMessage>("Keypoints");

    out_des = node_modifier.addOutput<DescriptorMessage>("Descriptors");
}

void ExtractDescriptors::process()
{
    if (refresh_) {
        refresh_ = false;

        std::string method = readParameter<std::string>("method");
        Extractor::Ptr next = ExtractorFactory::create("", method, param::StaticParameterProvider(getParameters()));

        extractor = next;
    }

    if (!extractor) {
        node_modifier_->setError("no extractor set");
        return;
    }

    node_modifier_->setNoError();

    CvMatMessage::ConstPtr img_msg = msg::getMessage<CvMatMessage>(in_img);

    DescriptorMessage::Ptr des_msg(new DescriptorMessage);

    // need to clone keypoints, extractDescriptors will modify the vector
    KeypointMessage::Ptr key_msg = msg::getClonedMessage<KeypointMessage>(in_key);

    extractor->extractDescriptors(img_msg->value, key_msg->value, des_msg->value);

    msg::publish(out_des, des_msg);
}

void ExtractDescriptors::update()
{
    refresh_ = true;
}
