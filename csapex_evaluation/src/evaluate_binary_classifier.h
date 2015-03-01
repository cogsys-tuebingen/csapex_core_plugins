#ifndef EVALUATE_BINARY_CLASSIFIER_H
#define EVALUATE_BINARY_CLASSIFIER_H

/// COMPONENT
#include <csapex_evaluation/confusion_matrix_message.h>

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class EvaluateBinaryClassifier : public csapex::Node
{
public:
    struct Metric
    {
        Metric(const std::string& name, const std::string& description,
               double worst, double best,
               double value)
            : name(name), description(description), worst(worst), best(best), value(value)
        {
        }

        std::string name;
        std::string description;
        double worst;
        double best;
        double value;
    };

    typedef std::vector<Metric> Metrics;

public:
    EvaluateBinaryClassifier();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    const Metrics& getMetrics() const;

public:
    boost::signals2::signal<void()> display_request;

private:
    Input* in_;

    Metrics metrics_;
};


}

#endif // EVALUATE_BINARY_CLASSIFIER_H
