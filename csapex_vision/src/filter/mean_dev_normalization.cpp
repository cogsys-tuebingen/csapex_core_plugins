/// HEADER
#include "mean_dev_normalization.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::MeanStdDevNormalization, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MeanStdDevNormalization::MeanStdDevNormalization()
{
}

namespace {
inline void normalize(const cv::Mat             &src,
                      const std::vector<double> &mean,
                      const std::vector<double> &dev,
                      cv::Mat                   &dst)
{
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for(unsigned int i = 0 ; i < channels.size() ; ++i) {
        channels.at(i) -= mean.at(i);
        channels.at(i) /= dev.at(i);
    }
    cv::merge(channels, dst);
}

inline void normalizeC(const cv::Mat             &src,
                       const std::vector<double> &mean,
                       const std::vector<double> &dev,
                       cv::Mat                   &dst)
{
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for(unsigned int i = 0 ; i < channels.size() ; ++i) {
        cv::subtract(channels.at(i),mean.at(i), channels.at(i), cv::Mat(), CV_32F);
        channels.at(i) /= dev.at(i);
    }
    cv::merge(channels, dst);
}

typedef std::shared_ptr< std::vector<double > const> VectorPtr;
}

void MeanStdDevNormalization::process()
{
    CvMatMessage::ConstPtr in_mat  = msg::getMessage<CvMatMessage>(in_mat_);
    VectorPtr              in_mean = msg::getMessage<GenericVectorMessage, double>(in_mean_);
    VectorPtr              in_dev  = msg::getMessage<GenericVectorMessage, double>(in_dev_);
    CvMatMessage::Ptr      out(new CvMatMessage(in_mat->getEncoding(), in_mat->frame_id, in_mat->stamp_micro_seconds));

    unsigned int c = in_mat->value.channels();


    if(c != in_mean->size()) {
        throw std::runtime_error("Channels != Mean dimension!");
    }

    if(c != in_dev->size()) {
        throw std::runtime_error("Channesl != Deviation dimension!");
    }
    int depth = in_mat->value.depth();
    if(depth == CV_64F || depth == CV_32F)
        normalize(in_mat->value, *in_mean, *in_dev,
                  out->value);
    else
        normalizeC(in_mat->value, *in_mean, *in_dev,
                   out->value);

    msg::publish(out_, out);
}

void MeanStdDevNormalization::setup(NodeModifier& node_modifier)
{
    in_mean_ = node_modifier.addInput<GenericVectorMessage, double>("Mean");
    in_dev_  = node_modifier.addInput<GenericVectorMessage, double>("StdDeviation");
    in_mat_  = node_modifier.addInput<CvMatMessage>("Matrix");
    out_     = node_modifier.addOutput<CvMatMessage>("Normalized Matrix");
}

void MeanStdDevNormalization::setupParameters(Parameterizable& parameters)
{
}
