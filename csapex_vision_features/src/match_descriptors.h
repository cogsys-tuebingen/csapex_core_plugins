#ifndef MATCH_DESCRIPTORS_H
#define MATCH_DESCRIPTORS_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_vision_features/descriptor_message.h>
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/match_message.h>

namespace csapex
{
class MatchDescriptors : public csapex::Node
{
private:
    enum Method
    {
        SIMPLE = 0,
        PEAK = 1,
        ROBUST = 2
    };

public:
    MatchDescriptors();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

private:
    void update();

    void match(connection_types::CvMatMessage::ConstPtr image1, connection_types::CvMatMessage::ConstPtr image2, connection_types::KeypointMessage::ConstPtr keypoints1,
               connection_types::KeypointMessage::ConstPtr keypoints2, connection_types::DescriptorMessage::ConstPtr descriptors1, connection_types::DescriptorMessage::ConstPtr descriptors2,
               std::vector<std::vector<cv::DMatch>>& matches);

    void matchRobust(connection_types::CvMatMessage::ConstPtr image1, connection_types::CvMatMessage::ConstPtr image2, connection_types::KeypointMessage::ConstPtr keypoints1,
                     connection_types::KeypointMessage::ConstPtr keypoints2, connection_types::DescriptorMessage::ConstPtr descriptors1, connection_types::DescriptorMessage::ConstPtr descriptors2,
                     std::vector<std::vector<cv::DMatch>>& matches);

    void matchSimple(connection_types::CvMatMessage::ConstPtr image1, connection_types::CvMatMessage::ConstPtr image2, connection_types::KeypointMessage::ConstPtr keypoints1,
                     connection_types::KeypointMessage::ConstPtr keypoints2, connection_types::DescriptorMessage::ConstPtr descriptors1, connection_types::DescriptorMessage::ConstPtr descriptors2,
                     std::vector<std::vector<cv::DMatch>>& matches);

    void matchPeak(connection_types::CvMatMessage::ConstPtr image1, connection_types::CvMatMessage::ConstPtr image2, connection_types::KeypointMessage::ConstPtr keypoints1,
                   connection_types::KeypointMessage::ConstPtr keypoints2, connection_types::DescriptorMessage::ConstPtr descriptors1, connection_types::DescriptorMessage::ConstPtr descriptors2,
                   std::vector<std::vector<cv::DMatch>>& matches);

private:
    Input* in_img_1;
    Input* in_key_1;
    Input* in_des_1;

    Input* in_img_2;
    Input* in_key_2;
    Input* in_des_2;

    Output* out_img;
    Output* out_match;

    Method current_method_;
};

}  // namespace csapex

#endif  // MATCH_DESCRIPTORS_H
