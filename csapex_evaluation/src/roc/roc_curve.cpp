/// HEADER
#include "roc_curve.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/factory/message_factory.h>
#include <csapex/profiling/timer.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/any_message.h>
#include <csapex/param/parameter_factory.h>
#include <fstream>

CSAPEX_REGISTER_CLASS(csapex::ROCCurve, csapex::Node)

using namespace csapex;

ROCCurve::ROCCurve()
    : in_confusion_(nullptr)
{
}

void ROCCurve::setup(NodeModifier& node_modifier)
{
    in_confusion_ = node_modifier.addInput<connection_types::ConfusionMatrixMessage>("Anything");
    in_threshold_ = node_modifier.addOptionalInput<double>("Discriminization\nThreshold");

    out_auc_ = node_modifier.addOutput<double>("AUC");
}

void ROCCurve::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareTrigger("reset"), [&](csapex::param::Parameter*) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        entries_.clear();
    });

    std::map<std::string, int> type = {
        {"ROC", (int) Type::ROC},
        {"Precision/Recall", (int) Type::PR}
    };

    parameters.addParameter(csapex::param::factory::declareParameterSet("type",
                                                                                 csapex::param::ParameterDescription("Diagram type."),
                                                                                 type,
                                                                                 (int) Type::ROC),
                            [this](param::Parameter* p) {
        type_ = static_cast<Type>(p->as<int>());
    });


    parameters.addParameter(param::factory::declareRange
                            ("width",
                             16, 4096, 128, 1));
    parameters.addParameter(param::factory::declareRange
                            ("height",
                             16, 4096, 128, 1));
    parameters.addParameter(param::factory::declareFileOutputPath("save to", "",  "*.txt"),
                            out_path_);
    parameters.addParameter(param::factory::declareTrigger("save"),
                            std::bind(&ROCCurve::saveData, this));
}



void ROCCurve::process()
{
    connection_types::ConfusionMatrixMessage::ConstPtr message = msg::getMessage<connection_types::ConfusionMatrixMessage>(in_confusion_);

    {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        ConfusionMatrix cm = message->confusion;

        if(cm.classes.size() != 2) {
            throw std::logic_error(std::string("needs a confusion matrix with exactly 2 classes, has ") + std::to_string(cm.classes.size()));
        }


        double threshold = cm.threshold;
        if(msg::hasMessage(in_threshold_)) {
            threshold = msg::getValue<double>(in_threshold_);
            cm.threshold = threshold;
        }

        int tp = cm.histogram[std::make_pair(1, 1)];
        int tn = cm.histogram[std::make_pair(0, 0)];
        int fp = cm.histogram[std::make_pair(0, 1)];
        int fn = cm.histogram[std::make_pair(1, 0)];
        int p = tp + fn;
        int n = tn + fp;

        if(n == 0) n = 1;
        if(p == 0) p = 1;

        Entry entry;
        entry.threshold = threshold;
        entry.recall = tp / (double) p;
        entry.specificity = fp / (double) n;
        entry.precision = tp / (double) (tp + fp);

        entries_[entry.threshold] = entry;
    }


    display_request();

    // TODO!
    if(msg::isConnected(out_auc_)) {
        double auc = 0.0;

        // approximate the area:
        /**             +  point b
         *             /|
         *            / |
         * point a   +  |
         *           |  |
         *           |  |
         *           +--+
         */

        std::vector<ROCCurve::Entry> entries = getEntries();
        if(entries.size() >= 2) {
            std::vector<ROCCurve::Entry> sorted;
            sorted.reserve(entries.size() + 2);

            ROCCurve::Entry first;
            first.specificity = 0.0;
            first.recall = 0.0;
            ROCCurve::Entry last;
            last.specificity = 1.0;
            last.recall = 1.0;

            sorted.push_back(first);
            sorted.insert(sorted.end(), entries.begin(), entries.end());
            sorted.push_back(last);


            ROCCurve::Entry* a = &sorted.front();
            ROCCurve::Entry* b = a + 1;

            ROCCurve::Entry* end = &sorted.back();
            for(; a != end; ++b, ++a) {
                double dx = std::abs(b->specificity - a->specificity);
                double dy = (b->recall + a->recall) / 2.0;

                double area = dx * dy;
                auc += area;
            }
        }


        msg::publish(out_auc_, auc);
    }
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

void ROCCurve::saveData()
{
    if(type_ == Type::ROC) {
        std::ofstream out(out_path_);
        for(const auto &e : entries_) {
            out << e.second.specificity << " "
                << e.second.recall << " "
                << e.second.threshold << std::endl;
        }
    } else if(type_ == Type::PR) {
        std::ofstream out(out_path_);
        for(const auto &e : entries_) {
            out << e.second.precision << " "
                << e.second.recall << " "
                << e.second.threshold << std::endl;

        }

    }
}
