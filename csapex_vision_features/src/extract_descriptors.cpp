/// HEADER
#include "extract_descriptors.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <utils_vision/utils/extractor.h>
#include <utils_vision/utils/extractor_factory.h>
#include <utils_vision/utils/extractor_manager.h>
#include <utils_param/range_parameter.h>
#include <utils_param/io.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/utility/q_signal_relay.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::ExtractDescriptors, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractDescriptors::ExtractDescriptors()
{
    addTag(Tag::get("Features"));

    ExtractorManager& manager = ExtractorManager::instance();
    std::vector<std::string> methods;

    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    Q_FOREACH(Pair fc, manager.descriptorExtractors()) {
        std::string key = fc.second.getType();
        methods.push_back(key);
    }

    param::Parameter::Ptr method = param::ParameterFactory::declareParameterStringSet("method", methods);
    addParameter(method, boost::bind(&ExtractDescriptors::update, this));

    Q_FOREACH(Pair fc, manager.descriptorExtractors()) {
        std::string key = fc.second.getType();
        boost::function<bool()> condition = (boost::bind(&param::Parameter::as<std::string>, method.get()) == key);

        Q_FOREACH(param::Parameter::Ptr param, manager.featureDescriptorParameters(key)) {
            param::Parameter::Ptr param_clone = param::ParameterFactory::clone(param);
            addConditionalParameter(param_clone, condition, boost::bind(&ExtractDescriptors::update, this));
        }
    }
}

void ExtractDescriptors::setup()
{
    setSynchronizedInputs(true);

    in_img = addInput<CvMatMessage>("Image");
    in_key = addInput<KeypointMessage>("Keypoints");

    out_des = addOutput<DescriptorMessage>("Descriptors");

    update();
}


void ExtractDescriptors::process()
{
    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    setError(false);

    CvMatMessage::Ptr img_msg = in_img->getMessage<CvMatMessage>();

    DescriptorMessage::Ptr des_msg(new DescriptorMessage);

    {
        QMutexLocker lock(&extractor_mutex);
        KeypointMessage::Ptr key_msg = in_key->getMessage<KeypointMessage>();

        extractor->extractDescriptors(img_msg->value, key_msg->value, des_msg->value);
    }

    out_des->publish(des_msg);
}
void ExtractDescriptors::update()
{
    std::string method = param<std::string>("method");
    Extractor::Ptr next = ExtractorFactory::create("", method, param::StaticParameterProvider(getParameters()));

    QMutexLocker lock(&extractor_mutex);
    extractor = next;
}
