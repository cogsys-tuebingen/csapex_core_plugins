/// HEADER
#include "mean_dev.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/vector_message.h>

CSAPEX_REGISTER_CLASS(csapex::MeanStdDev, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MeanStdDev::MeanStdDev()
{
}

void MeanStdDev::setup()
{
    in_mat_ = modifier_->addInput<CvMatMessage>("input");
    in_mask_ = modifier_->addOptionalInput<CvMatMessage>("input mask");
    out_mean_ = modifier_->addOutput<GenericVectorMessage, double>("mean");
    out_stddev_ = modifier_->addOutput<GenericVectorMessage, double>("stddev");

}

void MeanStdDev::setupParameters()
{

}

void MeanStdDev::process()
{
    CvMatMessage::Ptr in = in_mat_->getMessage<CvMatMessage>();
    boost::shared_ptr<std::vector<double> > out_mean(new std::vector<double>);
    boost::shared_ptr<std::vector<double> > out_stddev(new std::vector<double>);

    if(in_mask_->hasMessage()) {
        CvMatMessage::Ptr mask = in_mask_->getMessage<CvMatMessage>();
        cv::meanStdDev(in->value, *out_mean, *out_stddev,
                       mask->value);
    } else {
        cv::meanStdDev(in->value, *out_mean, *out_stddev);
    }

    out_mean_->publish<GenericVectorMessage, double>(out_mean);
    out_stddev_->publish<GenericVectorMessage, double>(out_stddev);
}
