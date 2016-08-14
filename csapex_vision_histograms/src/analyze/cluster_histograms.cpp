/// HEADER
#include "cluster_histograms.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <cslibs_vision/utils/histogram.hpp>
#include <csapex_vision_histograms/histogram_msg.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/assert.h>

using namespace vision_plugins;
using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(vision_plugins::ClusterHistograms, csapex::Node)


ClusterHistograms::ClusterHistograms()
{

}

namespace {

template <typename _Tp>
struct Dispatch {
    inline static
    void apply(const cv::Mat &src,
               const cv::Mat &mask,
               const cv::Mat &clusters,
               const cslibs_vision::histogram::Ranged &range,
               const int bins,
               const bool append,
               int  &num_clusters,
               HistogramMessage &msg)
    {
        std::vector<cv::Mat> channels;
        cv::split(src, channels);
        num_clusters = cslibs_vision::histogram::numClusters(clusters);

        if(append) {
            msg.value.ranges.resize(1, range);
            msg.value.histograms.resize(num_clusters);

            for(cv::Mat &m : msg.value.histograms) {
                m = cv::Mat(bins * channels.size(), 1, CV_32SC1, cv::Scalar::all(0));
            }

            for(int i = 0 ; i < channels.size() ; ++i) {
                std::vector<cv::Mat> tmp;
                cslibs_vision::histogram::histogram<_Tp>(channels.at(i), mask, clusters,
                                                        range.first,  range.second, bins,
                                                        num_clusters, tmp);

                for(int j = 0 ; j < num_clusters ; ++j) {
                    cv::Mat &histgram = msg.value.histograms.at(j);
                    cv::Mat histogram_roi(histgram, cv::Rect(0, i * bins, 1, bins));
                    tmp.at(j).copyTo(histogram_roi);
                }
            }
        } else {
            msg.value.ranges.resize(channels.size(), range);
            for(int i = 0 ; i < channels.size() ; ++i) {
                std::vector<cv::Mat> tmp;
                cslibs_vision::histogram::histogram<_Tp>(channels.at(i), mask, clusters,
                                                        range.first,  range.second, bins,
                                                        num_clusters, tmp);
                msg.value.histograms.insert(msg.value.histograms.end(), tmp.begin(), tmp.end());

            }
        }
    }
};
}

void ClusterHistograms::process()
{
    CvMatMessage::ConstPtr in       = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::ConstPtr clusters = msg::getMessage<CvMatMessage>(clusters_);

    cv::Mat mask;
    if(msg::hasMessage(mask_)) {
        CvMatMessage::ConstPtr mask_ptr = msg::getMessage<CvMatMessage>(mask_);
        mask = mask_ptr->value;
    }

    HistogramMessage::Ptr             out_histograms(new HistogramMessage);
    std::shared_ptr<std::vector<int>> out_labels(new std::vector<int>);

    if(clusters->value.type() != CV_32SC1) {
        throw std::runtime_error("Cluster label matrix must be single channel integer!");
    }
    if(!mask.empty() &&  mask.type() != CV_8UC1) {
        throw std::runtime_error("Mask must be single channel uchar!");
    }
    if(!mask.empty() &&
            (in->value.rows != mask.rows || in->value.cols != mask.cols)) {
        throw std::runtime_error("Mask dimension not matching!");
    }
    if(clusters->value.rows != in->value.rows ||
            clusters->value.cols != in->value.cols) {
        throw std::runtime_error("Cluster matrix dimension not matching!");
    }



    /// PUT INTRESTING CODE HERE
    bool   min_max  = readParameter<bool>("min max norm");
    int    bins     = readParameter<int>("bins");
    int    out_clusters = 0;
    bool   set_max = readParameter<bool>("set max");
    bool   set_min = readParameter<bool>("set min");
    bool   append = readParameter<bool>("append histograms");
    double max = 0.0;
    double min = 0.0;
    if(set_max)
        max = readParameter<double>("max");
    if(set_min)
        min = readParameter<double>("min");

    cslibs_vision::histogram::Ranged range;
    int type = in->value.type() & 7;
    switch(type) {
    case CV_8U:
        if(min_max)
            range = cslibs_vision::histogram::
                    make_min_max_range<unsigned char>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<unsigned char>();
        if(set_max)
            range.second = max;
        if(set_min)
            range.first = min;

        Dispatch<uchar>::apply(in->value,
                               mask,
                               clusters->value,
                               range,
                               bins,
                               append,
                               out_clusters,
                               *out_histograms);
        break;
    case CV_8S:
        if(min_max)
            range = cslibs_vision::histogram::
                    make_min_max_range<signed char>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<signed char>();

        if(set_max)
            range.second = max;
        if(set_min)
            range.first = min;

        Dispatch<char>::apply(in->value,
                              mask,
                              clusters->value,
                              range,
                              bins,
                              append,
                              out_clusters,
                              *out_histograms);
        break;
    case CV_16U:
        if(min_max)
            range = cslibs_vision::histogram::
                    make_min_max_range<unsigned short>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<unsigned short>();

        if(set_max)
            range.second = max;
        if(set_min)
            range.first = min;

        Dispatch<ushort>::apply(in->value,
                                mask,
                                clusters->value,
                                range,
                                bins,
                                append,
                                out_clusters,
                                *out_histograms);
        break;
    case CV_16S:
        if(min_max)
            range = cslibs_vision::histogram::
                    make_min_max_range<signed short>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<signed short>();
        if(set_max)
            range.second = max;
        if(set_min)
            range.first = min;

        Dispatch<short>::apply(in->value,
                               mask,
                               clusters->value,
                               range,
                               bins,
                               append,
                               out_clusters,
                               *out_histograms);
        break;
    case CV_32S:
        if(min_max)
            range = cslibs_vision::histogram::
                    make_min_max_range<int>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<int>();
        if(set_max)
            range.second = max;
        if(set_min)
            range.first = min;

        Dispatch<int>::apply(in->value,
                             mask,
                             clusters->value,
                             range,
                             bins,
                             append,
                             out_clusters,
                             *out_histograms);
        break;
    case CV_32F:
        if(min_max)
            range = cslibs_vision::histogram::
                    make_min_max_range<float>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<float>();
        if(set_max)
            range.second = max;
        if(set_min)
            range.first = min;

        Dispatch<float>::apply(in->value,
                               mask,
                               clusters->value,
                               range,
                               bins,
                               append,
                               out_clusters,
                               *out_histograms);

        break;
    default:
        throw std::runtime_error("Unsupported cv type!");
    }

    msg::publish(out_histograms_, out_histograms);
    msg::publish<int>(out_clusters_, out_clusters);

}

void ClusterHistograms::setup(NodeModifier &node_modifier)
{
    input_    = node_modifier.addInput<CvMatMessage>("input");
    clusters_ = node_modifier.addInput<CvMatMessage>("labels");
    mask_     = node_modifier.addOptionalInput<CvMatMessage>("mask");

    out_histograms_  = node_modifier.addOutput<HistogramMessage>("histograms");
    out_clusters_    = node_modifier.addOutput<int>("clusters");
}

void ClusterHistograms::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("min max norm", false));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("append histograms", false));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("bins", 2, 512, 256, 1));

    csapex::param::Parameter::Ptr set_max = csapex::param::ParameterFactory::declareBool("set max", false);
    parameters.addParameter(set_max);
    csapex::param::Parameter::Ptr set_min = csapex::param::ParameterFactory::declareBool("set min", false);
    parameters.addParameter(set_min);

    std::function<bool()> cond_max = [set_max]() { return set_max->as<bool>();};
    std::function<bool()> cond_min = [set_min]() { return set_min->as<bool>();};

    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("max", 255.0), cond_max);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("min", 0.0), cond_min);
}

