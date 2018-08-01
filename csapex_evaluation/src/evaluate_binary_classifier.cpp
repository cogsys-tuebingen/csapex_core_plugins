/// HEADER
#include "evaluate_binary_classifier.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::EvaluateBinaryClassifier, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


EvaluateBinaryClassifier::EvaluateBinaryClassifier()
{
}

void EvaluateBinaryClassifier::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareValue("positive class label", 1),
                            positive_class_label_);
    parameters.addParameter(param::factory::declareValue("negative class label", 0),
                            negative_class_label_);

}

void EvaluateBinaryClassifier::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<ConfusionMatrixMessage>("Confusion Matrix");
}

void EvaluateBinaryClassifier::process()
{
    connection_types::ConfusionMatrixMessage::ConstPtr msg = msg::getMessage<connection_types::ConfusionMatrixMessage>(in_);

    const ConfusionMatrix& cm = msg->confusion;

    if (cm.classes.size() < 2)
    {
        node_modifier_->setWarning("The confusion matrix must have at least 2 classes");
        return;
    }
    else if (cm.classes.size() > 2)
        node_modifier_->setWarning("The confusion matrix should only have 2 classes. Results may be inaccurate.");
    else
        node_modifier_->setNoError();

    {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        metrics_.clear();

        int tp = cm.histogram.at(std::make_pair(positive_class_label_, positive_class_label_));
        int tn = cm.histogram.at(std::make_pair(negative_class_label_, negative_class_label_));
        int fp = cm.histogram.at(std::make_pair(negative_class_label_, positive_class_label_));
        int fn = cm.histogram.at(std::make_pair(positive_class_label_, negative_class_label_));
        int p = tp + fn;
        int n = tn + fp;

        metrics_.push_back(Metric("Sensitivity", "True positive rate, hit rate, recall", 0.0, 1.0,
                                  tp / double(p)));
        metrics_.push_back(Metric("specificity", "(SPC) or True Negative Rate", 0.0, 1.0,
                                  tn / double(n)));
        metrics_.push_back(Metric("precision", "positive predictive value (PPV)", 0.0, 1.0,
                                  tp / double(tp + fp)));
        metrics_.push_back(Metric("negative predictive value", "NPV", 0.0, 1.0,
                                  tn / double(tn + fn)));
        metrics_.push_back(Metric("fall-out", "false positive rate (FPR)", 1.0, 0.0,
                                  fp / double(n)));
        metrics_.push_back(Metric("false discovery rate", "FDR", 1.0, 0.0,
                                  fp / double(fp + tp)));
        metrics_.push_back(Metric("Miss Rate", "False Negative Rate (FNR)", 1.0, 0.0,
                                  fn / double(p)));

        metrics_.push_back(Metric("accuracy", "ACC", 0.0, 1.0,
                                  (tp + tn) / double(p + n)));
        metrics_.push_back(Metric("F1 score", "the harmonic mean of precision and sensitivity", 0.0, 1.0,
                                  (2 * tp) / double(2 * tp + fp + fn)));
        metrics_.push_back(Metric("Matthews correlation coefficient", "MCC", -1.0, 1.0,
                                  (tp * tn - fp * fn)
                                  /
                                  std::sqrt(double(tp + fp) * double(tp + fn) * double(tn + fp) * double(tn + fn))));
    }

    display_request();
}

EvaluateBinaryClassifier::Metrics EvaluateBinaryClassifier::getMetrics() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return metrics_;
}
