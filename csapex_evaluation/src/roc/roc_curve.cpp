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

    std::map<std::string, int> type = {
        {"ROC", (int) Type::ROC},
        {"Precision/Recall", (int) Type::PR}
    };

    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet("type",
                                                                                 csapex::param::ParameterDescription("Diagram type."),
                                                                                 type,
                                                                                 (int) Type::ROC),
                            [this](param::Parameter* p) {
        type_ = static_cast<Type>(p->as<int>());
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

        double threshold = msg::getValue<double>(in_threshold_);

        if(!std::isnan(cm.threshold) && cm.threshold != threshold) {
            throw std::logic_error("threshold is wrong...");
        }

        int tp = cm.histogram.at(std::make_pair(1, 1));
        int tn = cm.histogram.at(std::make_pair(0, 0));
        int fp = cm.histogram.at(std::make_pair(0, 1));
        int fn = cm.histogram.at(std::make_pair(1, 0));
        int p = tp + fn;
        int n = tn + fp;

        Entry entry;
        entry.threshold = threshold;
        entry.recall = tp / (double) p;
        entry.specificity = fp / (double) n;
        entry.precision = tp / (double) (tp + fp);

        entries_[entry.threshold] = entry;
    }

    display_request();
}

std::vector<ROCCurve::Entry> ROCCurve::getEntries() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    std::vector<ROCCurve::Entry> res;
    res.reserve(entries_.size());

    for(const auto& p : entries_) {
        res.push_back(p.second);
    }

    std::sort(res.begin(), res.end());

    return res;
}

ROCCurve::Type ROCCurve::getType() const
{
    return type_;
}
