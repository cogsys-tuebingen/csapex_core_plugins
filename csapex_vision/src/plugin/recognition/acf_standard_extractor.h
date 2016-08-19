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

    cslibs_vision::ACFStandard::Parameters acf_params;
    cv::Size                               extraction_window_size;
    bool                                   mirror_;



};
}

#endif // ACFSTANDARDEXTRACTOR_H
