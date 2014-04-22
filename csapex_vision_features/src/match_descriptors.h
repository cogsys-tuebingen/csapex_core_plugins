#ifndef MATCH_DESCRIPTORS_H
#define MATCH_DESCRIPTORS_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

namespace csapex
{
class MatchDescriptors : public csapex::Node
{
private:
    enum Method {
        SIMPLE = 0,
        PEAK = 1,
        ROBUST = 2
    };

public:
    MatchDescriptors();

    virtual void setup();
    virtual void process();

private:
    void update();

    void match(connection_types::CvMatMessage::Ptr image1,
               connection_types::CvMatMessage::Ptr image2,
               connection_types::KeypointMessage::Ptr keypoints1,
               connection_types::KeypointMessage::Ptr keypoints2,
               connection_types::DescriptorMessage::Ptr descriptors1,
               connection_types::DescriptorMessage::Ptr descriptors2,
               std::vector<std::vector<cv::DMatch> > &matches);

    void matchRobust(connection_types::CvMatMessage::Ptr image1,
                     connection_types::CvMatMessage::Ptr image2,
                     connection_types::KeypointMessage::Ptr keypoints1,
                     connection_types::KeypointMessage::Ptr keypoints2,
                     connection_types::DescriptorMessage::Ptr descriptors1,
                     connection_types::DescriptorMessage::Ptr descriptors2,
                     std::vector<std::vector<cv::DMatch> > &matches);

    void matchSimple(connection_types::CvMatMessage::Ptr image1,
                     connection_types::CvMatMessage::Ptr image2,
                     connection_types::KeypointMessage::Ptr keypoints1,
                     connection_types::KeypointMessage::Ptr keypoints2,
                     connection_types::DescriptorMessage::Ptr descriptors1,
                     connection_types::DescriptorMessage::Ptr descriptors2,
                     std::vector<std::vector<cv::DMatch> > &matches);

    void matchPeak(connection_types::CvMatMessage::Ptr image1,
                   connection_types::CvMatMessage::Ptr image2,
                   connection_types::KeypointMessage::Ptr keypoints1,
                   connection_types::KeypointMessage::Ptr keypoints2,
                   connection_types::DescriptorMessage::Ptr descriptors1,
                   connection_types::DescriptorMessage::Ptr descriptors2,
                   std::vector<std::vector<cv::DMatch> > &matches);

private:
    ConnectorIn* in_img_1;
    ConnectorIn* in_key_1;
    ConnectorIn* in_des_1;

    ConnectorIn* in_img_2;
    ConnectorIn* in_key_2;
    ConnectorIn* in_des_2;

    ConnectorOut* out_img;

    Method current_method_;
};

} /// NAMESPACE

#endif // MATCH_DESCRIPTORS_H
