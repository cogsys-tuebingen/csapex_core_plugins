#ifndef ML_EVALUATOR_H
#define ML_EVALUATOR_H

/// COMPONENT
#include <csapex_ml/features_message.h>
#include <csapex_evaluation/confusion_matrix_message.h>

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {


class MLEvaluator : public csapex::Node
{

    struct BinaryClassificationResult
    {
        int p;
        int n;

        int tp;
        int tn;
        int fp;
        int fn;
    };

    struct BinaryClassificationMetrics
    {
        BinaryClassificationMetrics();
        BinaryClassificationMetrics(BinaryClassificationResult result);

        BinaryClassificationResult classification;

        union {
            double sensitivity;
            double true_positive_rate;
            double tpr;
            double hit_rate;
            double recall;
        };

        union {
            double specificity;
            double spc;
            double true_negative_rate;
            double tnr;
        };

        union {
            double precision;
            double positive_predictive_value;
            double ppv;
        };

        union {
            double negative_predictive_value;
            double npv;
        };

        union {
            double fall_out;
            double false_positive_rate;
            double fpr;
        };

        union {
            double false_discovery_rate;
            double fdr;
        };

        union {
            double negative_rate;
            double fnr;
        };

        union {
            double accuracy;
            double acc;
        };

        union {
            double f1_score;
            double f1s;
        };

        union {
            double matthews_correlation_coefficient;
            double mcc;
        };
    };

public:
    MLEvaluator();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* in_truth_;
    Input* in_classified_;
    Output* out_;

    ConfusionMatrix confusion_;
};


}

#endif // ML_EVALUATOR_H
