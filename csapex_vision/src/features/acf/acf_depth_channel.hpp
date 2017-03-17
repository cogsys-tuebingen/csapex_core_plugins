#pragma once

#include <csapex/model/node.h>

#include <csapex_opencv/roi.h>

namespace csapex { namespace vision {

class ACFDepthChannel : public csapex::Node
{
public:
    enum class Type { BINARY, TERNARY };
    enum class Method { MEDIAN, MEAN };

    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    void extractChannel(const cv::Mat& depth, cv::Mat& channel);
    void updateWindow();

private:
    Input*  in_image_;
    Input*  in_rois_;
    Output* out_channels_;

    int window_width_;
    int window_height_;
    double window_ratio_ = 0.0;
    bool keep_ratio_;
    bool mirror_;
    int block_size_;
    Type type_;
    Method method_;
    double threshold_;
};

}}
