/// HEADER
#include "extract_keypoints.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>

/// PROJECT
#include <utils_vision/utils/extractor.h>
#include <utils_vision/utils/extractor_factory.h>
#include <utils_vision/utils/extractor_manager.h>
#include <utils_param/range_parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/utility/q_signal_relay.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::ExtractKeypoints, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractKeypoints::ExtractKeypoints()
    : refresh_(true)
{
    ExtractorManager& manager = ExtractorManager::instance();
    std::vector<std::string> methods;

    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    Q_FOREACH(Pair fc, manager.featureDetectors()) {
        std::string key = fc.second.getType();
        methods.push_back(key);
    }

    param::Parameter::Ptr method = param::ParameterFactory::declareParameterStringSet("method", methods);
    addParameter(method, std::bind(&ExtractKeypoints::update, this));

    Q_FOREACH(Pair fc, manager.featureDetectors()) {
        std::string key = fc.second.getType();
        std::function<bool()> condition = [method, key]() { return method->as<std::string>() == key; };

        Q_FOREACH(param::Parameter::Ptr param, manager.featureDetectorParameters(key)) {
            param::Parameter::Ptr param_clone = param::ParameterFactory::clone(param);
            addConditionalParameter(param_clone, condition, std::bind(&ExtractKeypoints::update, this));
        }
    }
}

void ExtractKeypoints::setup()
{
    in_img = modifier_->addInput<CvMatMessage>("Image");
    in_mask = modifier_->addOptionalInput<CvMatMessage>("Mask");

    out_key = modifier_->addOutput<csapex::connection_types::KeypointMessage>("Keypoints");
}

void ExtractKeypoints::process()
{
    if(refresh_) {
        refresh_ = false;

        std::string method = readParameter<std::string>("method");
        Extractor::Ptr next = ExtractorFactory::create(method, "", param::StaticParameterProvider(getParameters()));

        extractor = next;
    }

    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    setError(false);

    CvMatMessage::ConstPtr img_msg = in_img->getMessage<CvMatMessage>();

    KeypointMessage::Ptr key_msg(new KeypointMessage);

    if(in_mask->hasMessage()) {
        CvMatMessage::ConstPtr mask_msg = in_mask->getMessage<CvMatMessage>();

        extractor->extractKeypoints(img_msg->value, mask_msg->value, key_msg->value);

    } else {
        extractor->extractKeypoints(img_msg->value, cv::Mat(), key_msg->value);
    }

    out_key->publish(key_msg);
}


void ExtractKeypoints::update()
{
    refresh_ = true;
}

