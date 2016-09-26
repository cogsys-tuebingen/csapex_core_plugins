#pragma once

/// PROJECT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node.h>


namespace csapex {
class CSAPEX_EXPORT_PLUGIN MakeVideo : public csapex::Node
{
public:
    MakeVideo();

    void setup(NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;
    void process() override;

private:
    Input *input_;

    std::string path_;
    std::map<int, std::string> file_extensions_;
    int         codec_type_;
    double      frame_rate_;
    cv::Size    frame_size_;
    bool        buffer_;
    cv::VideoWriter vw_;

    std::vector<connection_types::CvMatMessage::ConstPtr> msg_buffer_;

    void writeOnline(const connection_types::CvMatMessage::ConstPtr &msg);
    void writeBuffer();
    void clear();
};
}
