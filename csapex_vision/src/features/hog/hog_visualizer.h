#ifndef HOG_VISUALIZER_H
#define HOG_VISUALIZER_H

/// PROJECT
#include <csapex/model/node.h>

/// EXTRACT HOG FEATURE

namespace cv {
class Mat;
}

namespace csapex {
class HOGVisualizer : public csapex::Node
{
public:
    HOGVisualizer();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    csapex::Input  *in_img_;
    csapex::Input  *in_rois_;
    csapex::Input  *in_positive_svm_weights_;
    csapex::Input  *in_negative_svm_weights_;
    csapex::Input  *in_descriptors_;
    csapex::Output *out_hog_images_;
    csapex::Output *out_positive_block_visualizations_;
    csapex::Output *out_negative_block_visualizations_;

    bool            signed_gradient_;
    int             bins_;
    int             cells_x_;
    int             cells_y_;
    int             cell_size_;
    int             block_size_;
    int             block_stride_;
    double          scale_hog_image_;
    double          scale_block_images_;
    int             scale_gradient_;

    void renderHOGFeatures(const cv::Mat &src,
                           const std::vector<float> &descriptor,
                           cv::Mat &dst);


};
}
#endif // HOG_VISUALIZER_H
