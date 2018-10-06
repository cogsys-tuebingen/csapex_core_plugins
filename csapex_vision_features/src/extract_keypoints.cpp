/// HEADER
#include "extract_keypoints.h"

/// COMPONENT
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
#include <csapex/param/value_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <cslibs_vision/utils/extractor.h>

/// SYSTEM
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::ExtractKeypoints, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractKeypoints::ExtractKeypoints() : refresh_(true)
{
}

void ExtractKeypoints::setupParameters(Parameterizable& parameters)
{
    ExtractorManager& manager = ExtractorManager::instance();
    std::vector<std::string> methods;

    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    for (Pair fc : manager.featureDetectors()) {
        std::string key = fc.second.getType();
        methods.push_back(key);
    }

    csapex::param::Parameter::Ptr method = csapex::param::ParameterFactory::declareParameterStringSet("method", methods);
    parameters.addParameter(method, std::bind(&ExtractKeypoints::update, this));

    for (Pair fc : manager.featureDetectors()) {
        std::string key = fc.second.getType();
        std::function<bool()> condition = [method, key]() { return method->as<std::string>() == key; };

        for (csapex::param::Parameter::Ptr param : manager.featureDetectorParameters(key)) {
            csapex::param::Parameter::Ptr param_clone = csapex::param::ParameterFactory::clone(param);
            parameters.addConditionalParameter(param_clone, condition, std::bind(&ExtractKeypoints::update, this));
        }
    }
}

void ExtractKeypoints::setup(NodeModifier& node_modifier)
{
    in_img = node_modifier.addInput<CvMatMessage>("Image");
    in_mask = node_modifier.addOptionalInput<CvMatMessage>("Mask");

    out_key = node_modifier.addOutput<csapex::connection_types::KeypointMessage>("Keypoints");
}

void ExtractKeypoints::process()
{
    if (refresh_) {
        refresh_ = false;

        std::string method = readParameter<std::string>("method");
        Extractor::Ptr next = ExtractorFactory::create(method, "", param::StaticParameterProvider(getParameters()));

        extractor = next;
    }

    if (!extractor) {
        node_modifier_->setError("no extractor set");
        return;
    }

    node_modifier_->setNoError();

    CvMatMessage::ConstPtr img_msg = msg::getMessage<CvMatMessage>(in_img);

    KeypointMessage::Ptr key_msg(new KeypointMessage);

    if (msg::hasMessage(in_mask)) {
        CvMatMessage::ConstPtr mask_msg = msg::getMessage<CvMatMessage>(in_mask);

        extractor->extractKeypoints(img_msg->value, mask_msg->value, key_msg->value);

    } else {
        extractor->extractKeypoints(img_msg->value, cv::Mat(), key_msg->value);
    }

    msg::publish(out_key, key_msg);
}

void ExtractKeypoints::update()
{
    refresh_ = true;
}
