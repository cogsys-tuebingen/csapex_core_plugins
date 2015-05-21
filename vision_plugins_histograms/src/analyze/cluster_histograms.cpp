/// HEADER
#include "cluster_histograms.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <utils_vision/utils/histogram.hpp>
#include <vision_plugins_histograms/histogram_msg.h>
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
               const utils_vision::histogram::Ranged &range,
               const int bins,
               int  &num_clusters,
               HistogramMessage &msg)
    {
        std::vector<cv::Mat> channels;
        cv::split(src, channels);
        num_clusters = utils_vision::histogram::numClusters(clusters);
        msg.value.ranges.resize(channels.size(), range);
        for(int i = 0 ; i < channels.size() ; ++i) {
            std::vector<cv::Mat> tmp;
            utils_vision::histogram::histogram<_Tp>(channels.at(i), mask, clusters,
                                                    range.first,  range.second, bins,
                                                    num_clusters, tmp);
            msg.value.histograms.insert(msg.value.histograms.end(), tmp.begin(), tmp.end());

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
    bool min_max  = readParameter<bool>("min max");
    int  bins     = readParameter<int>("bins");
    int  out_clusters = 0;

    utils_vision::histogram::Ranged range;
    int type = in->value.type() & 7;
    switch(type) {
    case CV_8U:
        if(min_max)
            range = utils_vision::histogram::
                    make_min_max_range<unsigned char>(in->value, mask);
        else
            range = utils_vision::histogram::
                    make_range<unsigned char>();
        Dispatch<uchar>::apply(in->value,
                               mask,
                               clusters->value,
                               range,
                               bins,
                               out_clusters,
                               *out_histograms);
        break;
    case CV_8S:
        if(min_max)
            range = utils_vision::histogram::
                    make_min_max_range<signed char>(in->value, mask);
        else
            range = utils_vision::histogram::
                    make_range<signed char>();
        Dispatch<char>::apply(in->value,
                              mask,
                              clusters->value,
                              range,
                              bins,
                              out_clusters,
                              *out_histograms);
        break;
    case CV_16U:
        if(min_max)
            range = utils_vision::histogram::
                    make_min_max_range<unsigned short>(in->value, mask);
        else
            range = utils_vision::histogram::
                    make_range<unsigned short>();
        Dispatch<ushort>::apply(in->value,
                                mask,
                                clusters->value,
                                range,
                                bins,
                                out_clusters,
                                *out_histograms);
        break;
    case CV_16S:
        if(min_max)
            range = utils_vision::histogram::
                    make_min_max_range<signed short>(in->value, mask);
        else
            range = utils_vision::histogram::
                    make_range<signed short>();
        Dispatch<short>::apply(in->value,
                               mask,
                               clusters->value,
                               range,
                               bins,
                               out_clusters,
                               *out_histograms);
        break;
    case CV_32S:
        if(min_max)
            range = utils_vision::histogram::
                    make_min_max_range<int>(in->value, mask);
        else
            range = utils_vision::histogram::
                    make_range<int>();
        Dispatch<int>::apply(in->value,
                             mask,
                             clusters->value,
                             range,
                             bins,
                             out_clusters,
                             *out_histograms);
        break;
    case CV_32F:
        if(min_max)
            range = utils_vision::histogram::
                    make_min_max_range<float>(in->value, mask);
        else
            range = utils_vision::histogram::
                    make_range<float>();
        Dispatch<float>::apply(in->value,
                               mask,
                               clusters->value,
                               range,
                               bins,
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
    parameters.addParameter(param::ParameterFactory::declareBool("min max", false));
    parameters.addParameter(param::ParameterFactory::declareRange("bins", 2, 512, 256, 1));
}

