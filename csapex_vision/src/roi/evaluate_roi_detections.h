#ifndef EVALUATEDETECTION_H
#define EVALUATEDETECTION_H

#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <opencv2/core/core.hpp>
#include <csapex_evaluation/confusion_matrix.h>

namespace csapex {
class EvaluateROIDetections : public csapex::Node
{
public:
    enum ClassificationType {BACKGROUND = 0, HUMAN = 1, HUMAN_PART = 2, UNKNOWN = 3};
    enum OverlapMode {OVERLAP_IOU, OVERLAP_MAX};

    EvaluateROIDetections();

    void setup(csapex::NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;
    void process() override;

private:
    enum Mode {IGNORE_PARTLY_VISIBLE, INTEGRATE_PARTLY_VISIBLE};

    std::string     path_of_statistic_;
    Mode            mode_;
    OverlapMode     overlap_mode_;

    double          percentage_of_overlap_;

    csapex::Input               *in_prediction_;
    csapex::Input               *in_groundtruth_;
    csapex::Output              *out_confusion_;
    csapex::Slot                *slot_save_;
    csapex::Slot                *slot_reset_;
    csapex::ConfusionMatrix      confusion_;
    std::array<std::size_t, 2>   human_parts_found_;
    std::size_t                  frame_count_;

    void save() const;
    void resetConfusion();

};
}

#endif // EVALUATEDETECTION_H
