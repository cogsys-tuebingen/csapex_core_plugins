/// HEADER
#include "to_feature.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_factory.h>
#include <csapex/model/connection_type.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/path_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/core/settings.h>

/// CONCRETE MESSAGES
#include <csapex_ml/features_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/descriptor_message.h>
#include <vision_plugins_histograms/histogram_msg.h>

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::ToFeature, csapex::Node)

ToFeature::ToFeature()
{
}

void ToFeature::setup()
{
    input_  = modifier_->addInput<connection_types::AnyMessage>("anything");
    output_ = modifier_->addOutput<GenericVectorMessage, FeaturesMessage>("features");
}

void ToFeature::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("class id", 0, 255, 0, 1));
}

namespace {
typedef boost::shared_ptr<std::vector<FeaturesMessage> > Features;

template<typename T>
inline void doProcessSingle(const T   &src,
                            boost::shared_ptr< std::vector<FeaturesMessage> >  &dst,
                            const int label = 0)
{
    std::runtime_error("Unsupported message type!");
}

template<>
inline void doProcessSingle(const typename HistogramMessage::ConstPtr &src,
                            boost::shared_ptr< std::vector<FeaturesMessage> > &dst,
                            const int label)
{
    FeaturesMessage features;
    for(std::vector<cv::Mat>::const_iterator
        it  = src->value.histograms.begin() ;
        it != src->value.histograms.end() ;
        ++it) {
        std::vector<float> tmp;
        it->copyTo(tmp);
        features.value.insert(features.value.end(),
                              tmp.begin(),
                              tmp.end());
    }
    features.classification = label;
    dst->push_back(features);
}

template<>
inline void doProcessSingle(const typename CvMatMessage::ConstPtr &src,
                            boost::shared_ptr< std::vector<FeaturesMessage> > &dst,
                            const int label)
{
    FeaturesMessage features;
    cv::Mat tmp;
    src->value.copyTo(tmp);
    tmp.convertTo(tmp, CV_32F);
    tmp = tmp.reshape(1, 1);
    tmp.copyTo(features.value);

    features.classification = label;
    dst->push_back(features);
}

template<>
inline void doProcessSingle(const typename DescriptorMessage::ConstPtr &src,
                            boost::shared_ptr< std::vector<FeaturesMessage> > &dst,
                            const int label)
{
    for(int i = 0 ; i < src->value.rows ; ++i) {
        FeaturesMessage features;
        src->value.row(i).copyTo(features.value);
        features.classification = label;
        dst->push_back(features);
    }
}

inline void processSingle(const ConnectionType::ConstPtr                    &src,
                          boost::shared_ptr< std::vector<FeaturesMessage> > &dst,
                          const int                                         label)
{
    CvMatMessage::ConstPtr      cv =
            boost::dynamic_pointer_cast<CvMatMessage const>(src);
    if(cv)
        doProcessSingle(cv, dst, label);

    DescriptorMessage::ConstPtr d  =
            boost::dynamic_pointer_cast<DescriptorMessage const>(src);
    if(d)
        doProcessSingle(d, dst, label);

    HistogramMessage::ConstPtr  h  =
            boost::dynamic_pointer_cast<HistogramMessage const>(src);
    if(h)
        doProcessSingle(h, dst, label);
}

inline void processVector(const connection_types::VectorMessage::ConstPtr    &src,
                          boost::shared_ptr< std::vector<FeaturesMessage> >  &dst,
                          const int label)
{
    for(std::size_t i = 0, total = src->value.size(); i < total; ++i) {
        processSingle(src->value[i], dst, label);
    }
}
}


void ToFeature::process()
{
    ConnectionType::ConstPtr msg = input_->getMessage<ConnectionType>();
    boost::shared_ptr< std::vector<FeaturesMessage> > out (new std::vector<FeaturesMessage>);


    int class_id = readParameter<int>("class id");

    connection_types::VectorMessage::ConstPtr vector =
            boost::dynamic_pointer_cast<connection_types::VectorMessage const>(msg);
    if(vector) {
        processVector(vector, out, class_id);
    } else {
        processSingle(msg, out, class_id);
    }

    output_->publish<GenericVectorMessage, FeaturesMessage>(out);
}

