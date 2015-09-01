#ifndef CLOUD_RENDERER_H
#define CLOUD_RENDERER_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex {

class CloudRenderer : public InteractiveNode
{
public:
    CloudRenderer();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

    virtual void beginProcess() override;
    virtual void finishProcess() override;

    void publishImage(const cv::Mat &img);

    connection_types::PointCloudMessage::ConstPtr getMessage() const;
    bool isOutputConnected() const;

private:
    void refresh();

private:
    Input* input_;
    Output* output_;

public:
    boost::signals2::signal<void()> display_request;
    boost::signals2::signal<void()> refresh_request;

private:
    mutable std::mutex message_mutex_;
    connection_types::PointCloudMessage::ConstPtr message_;

    connection_types::CvMatMessage::Ptr result_;
};

}

#endif // CLOUD_RENDERER_H
