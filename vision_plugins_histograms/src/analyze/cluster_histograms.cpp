/// HEADER
#include "cluster_histograms.h"

/// PROJECT
#include <csapex/msg/io.h>
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

void ClusterHistograms::process()
{
    CvMatMessage::ConstPtr in       = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::ConstPtr clusters = msg::getMessage<CvMatMessage>(clusters_);
    CvMatMessage::ConstPtr mask;
    if(msg::hasMessage(mask_)) {
        mask = msg::getMessage<CvMatMessage>(mask_);
    }
    HistogramMessage::Ptr             out_histograms(new HistogramMessage);
    std::shared_ptr<std::vector<int>> out_labels(new std::vector<int>);

    if(clusters->value.type() != CV_32SC1) {
        throw std::runtime_error("Cluster label matrix must be single channel integer!");
    }
    if(!mask->value.empty() && mask->value.type() != CV_8UC1) {
        throw std::runtime_error("Mask must be single channel uchar!");
    }

    /// PUT INTRESTING CODE HERE

    utils_vision::histogram::Range range;
    int type = in->value.type() & 7;
    switch(type) {
    case CV_8U:
        break;
    case CV_8S:
        break;
    case CV_16U:
        break;
    case CV_16S:
        break;
    case CV_32S:
        break;
    case CV_32F:
        break;
    case CV_64F:
        break;
    default:
        throw std::runtime_error("Unsupported cv type!");
    }




    msg::publish(out_histograms_, out_histograms);
    msg::publish<GenericVectorMessage, int>(out_cluster_ids_, out_labels);

}

void ClusterHistograms::setup(NodeModifier &node_modifier)
{
    input_    = node_modifier.addInput<CvMatMessage>("input");
    clusters_ = node_modifier.addInput<CvMatMessage>("labels");
    mask_     = node_modifier.addInput<CvMatMessage>("mask");

    out_cluster_ids_ = node_modifier.addOutput<GenericVectorMessage, int>("assignment");
    out_histograms_  = node_modifier.addOutput<HistogramMessage>("histograms");

    update();
}

void ClusterHistograms::setupParameters(Parameterizable &parameters)
{
    /// BINS
    /// MIN MAX
    /// ...
}

void ClusterHistograms::update()
{

}
