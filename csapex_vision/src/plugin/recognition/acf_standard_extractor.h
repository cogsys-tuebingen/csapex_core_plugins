#ifndef ACFSTANDARDEXTRACTOR_H
#define ACFSTANDARDEXTRACTOR_H

/// PROJECT
#include <csapex/model/node.h>
#include <cslibs_vision/features/acf.hpp>

namespace csapex {
class ACFStandardExtractor : public csapex::Node
{
public:

    void setupParameters(Parameterizable &parameters) override;
    void setup(NodeModifier &node_modifier) override;
    void process() override;


private:
    csapex::Input  *in_img_;
    csapex::Input  *in_rois_;
    csapex::Output *out_;

    cv::Size                               extraction_window_size;
    double                                 ratio_x_y_;
    bool                                   mirror_;
    bool                                   keep_ratio_;

    cslibs_vision::ACFStandard::Parameters acf_params;

    void update();

};
}

#endif // ACFSTANDARDEXTRACTOR_H
