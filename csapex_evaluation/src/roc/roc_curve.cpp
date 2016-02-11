/// HEADER
#include "roc_curve.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/factory/message_factory.h>
#include <csapex/utility/timer.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/any_message.h>
#include <csapex/param/parameter_factory.h>

CSAPEX_REGISTER_CLASS(csapex::ROCCurve, csapex::Node)

using namespace csapex;

ROCCurve::ROCCurve()
    : in_confusion_(nullptr)
{
}

void ROCCurve::setup(NodeModifier& node_modifier)
{
    in_confusion_ = node_modifier.addInput<connection_types::ConfusionMatrixMessage>("Anything");
    in_threshold_ = node_modifier.addInput<double>("Discriminization\nThreshold");

}

void ROCCurve::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("reset"), [&](csapex::param::Parameter*) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        entries_.clear();
    });


    parameters.addParameter(param::ParameterFactory::declareRange
                            ("width",
                             16, 4096, 128, 1));
    parameters.addParameter(param::ParameterFactory::declareRange
                            ("height",
                             16, 4096, 128, 1));
}



void ROCCurve::process()
{
    connection_types::ConfusionMatrixMessage::ConstPtr message = msg::getMessage<connection_types::ConfusionMatrixMessage>(in_confusion_);

    {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        const ConfusionMatrix& cm = message->confusion;

        if(cm.classes.size() != 2) {
            throw std::logic_error("needs a confusion matrix with exactly 2 classes");
        }

        int tp = cm.histogram.at(std::make_pair(1, 1));
        int tn = cm.histogram.at(std::make_pair(0, 0));
        int fp = cm.histogram.at(std::make_pair(0, 1));
        int fn = cm.histogram.at(std::make_pair(1, 0));
        int p = tp + fn;
        int n = tn + fp;

        Entry entry;
        entry.threshold = msg::getValue<double>(in_threshold_);
        entry.tpr = tp / (double) p;
        entry.fpr = fp / (double) n;

        entries_[entry.threshold] = entry;
    }

    display_request();
}

const std::map<double, ROCCurve::Entry>& ROCCurve::getEntries() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return entries_;
}

