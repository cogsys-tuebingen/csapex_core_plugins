#ifndef ACFEXTRACTOR_H
#define ACFEXTRACTOR_H

/// PROJECT
#include <csapex/model/node.h>
#include <cslibs_vision/features/acf.hpp>

namespace csapex
{
class ACFDynamicExtractor : public csapex::Node
{
public:
    ACFDynamicExtractor();

    void setupParameters(Parameterizable& parameters) override;
    void setup(NodeModifier& node_modifier) override;
    void process() override;

private:
    csapex::Input* in_img_;
    csapex::Input* in_rois_;
    csapex::Output* out_features_;

    cv::Size window_size_before_;
    cv::Size window_size_;
    double ratio_w_h_;
    bool mirror_;
    bool keep_ratio_;

    cslibs_vision::ACFDynamic::Parameters acf_params_;

    void updateWindow();
};
}  // namespace csapex

#endif  // ACFEXTRACTOR_H
