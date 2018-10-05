/// HEADER
#include "mean_dev.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::MeanStdDev, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MeanStdDev::MeanStdDev()
{
}

void MeanStdDev::setup(NodeModifier& node_modifier)
{
    in_mat_ = node_modifier.addInput<CvMatMessage>("input");
    in_mask_ = node_modifier.addOptionalInput<CvMatMessage>("input mask");
    out_mean_ = node_modifier.addOutput<GenericVectorMessage, double>("mean");
    out_stddev_ = node_modifier.addOutput<GenericVectorMessage, double>("stddev");
}

void MeanStdDev::setupParameters(Parameterizable& parameters)
{
}

void MeanStdDev::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(in_mat_);
    std::shared_ptr<std::vector<double>> out_mean(new std::vector<double>);
    std::shared_ptr<std::vector<double>> out_stddev(new std::vector<double>);

    if (msg::hasMessage(in_mask_)) {
        CvMatMessage::ConstPtr mask = msg::getMessage<CvMatMessage>(in_mask_);
        cv::meanStdDev(in->value, *out_mean, *out_stddev, mask->value);
    } else {
        cv::meanStdDev(in->value, *out_mean, *out_stddev);
    }

    msg::publish<GenericVectorMessage, double>(out_mean_, out_mean);
    msg::publish<GenericVectorMessage, double>(out_stddev_, out_stddev);
}
