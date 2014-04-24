/// HEADER
#include "extract_keypoints.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>

/// PROJECT
#include <utils/extractor.h>
#include <utils/extractor_factory.h>
#include <utils/extractor_manager.h>
#include <utils_param/range_parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/io.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/utility/q_signal_relay.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>

/// SYSTEM
#include <QFrame>
#include <csapex/utility/register_apex_plugin.h>
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::ExtractKeypoints, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractKeypoints::ExtractKeypoints()
{
    addTag(Tag::get("Features"));

    ExtractorManager& manager = ExtractorManager::instance();
    std::vector<std::string> methods;

    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    Q_FOREACH(Pair fc, manager.featureDetectors()) {
        std::string key = fc.second.getType();
        methods.push_back(key);
    }

    param::Parameter::Ptr method = param::ParameterFactory::declareParameterStringSet("method", methods);
    addParameter(method, boost::bind(&ExtractKeypoints::update, this));

    Q_FOREACH(Pair fc, manager.featureDetectors()) {
        std::string key = fc.second.getType();
        boost::function<bool()> condition = (boost::bind(&param::Parameter::as<std::string>, method.get()) == key);

        Q_FOREACH(param::Parameter::Ptr param, manager.featureDetectorParameters(key)) {
            param::Parameter::Ptr param_clone = param::ParameterFactory::clone(param);
            addConditionalParameter(param_clone, condition, boost::bind(&ExtractKeypoints::update, this));
        }
    }
}

void ExtractKeypoints::setup()
{
    setSynchronizedInputs(true);

    in_img = addInput<CvMatMessage>("Image");
    in_mask = addInput<CvMatMessage>("Mask", true);

    out_key = addOutput<csapex::connection_types::KeypointMessage>("Keypoints");

    update();
}

void ExtractKeypoints::process()
{
    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    setError(false);

    CvMatMessage::Ptr img_msg = in_img->getMessage<CvMatMessage>();

    KeypointMessage::Ptr key_msg(new KeypointMessage);

    {
        QMutexLocker lock(&extractor_mutex);
        if(in_mask->isConnected()) {
            CvMatMessage::Ptr mask_msg = in_mask->getMessage<CvMatMessage>();

            extractor->extractKeypoints(img_msg->value, mask_msg->value, key_msg->value);

        } else {
            extractor->extractKeypoints(img_msg->value, cv::Mat(), key_msg->value);
        }
    }

    out_key->publish(key_msg);
}


void ExtractKeypoints::update()
{
    std::string method = param<std::string>("method");
    Extractor::Ptr next = ExtractorFactory::create(method, "", param::StaticParameterProvider(getParameters()));

    QMutexLocker lock(&extractor_mutex);
    extractor = next;
}

