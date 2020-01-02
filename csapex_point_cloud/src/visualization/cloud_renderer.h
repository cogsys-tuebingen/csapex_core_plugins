#ifndef CLOUD_RENDERER_H
#define CLOUD_RENDERER_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class CloudRenderer : public InteractiveNode
{
public:
    CloudRenderer();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

    void beginProcess(csapex::NodeModifier& node_modifier, Parameterizable& parameters) override;
    void finishProcess(csapex::NodeModifier& node_modifier, Parameterizable& parameters) override;

    void publishImage(const cv::Mat& img);

    connection_types::PointCloudMessage::ConstPtr getMessage() const;
    bool isOutputConnected() const;

private:
    void refresh();

private:
    Input* input_;
    Output* output_;

public:
    slim_signal::Signal<void()> display_request;
    slim_signal::Signal<void()> refresh_request;

private:
    mutable std::mutex message_mutex_;
    connection_types::PointCloudMessage::ConstPtr message_;

    connection_types::CvMatMessage::Ptr result_;
};

}  // namespace csapex

#endif  // CLOUD_RENDERER_H
