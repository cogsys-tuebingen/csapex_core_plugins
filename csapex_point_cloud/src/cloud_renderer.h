#ifndef CLOUD_RENDERER_H
#define CLOUD_RENDERER_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex {

class CloudRenderer : public InteractiveNode
{
    friend class CloudRendererAdapter;

public:
    CloudRenderer();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();

    void publishImage(const cv::Mat &img);

private:
    void refresh();

private:
    Input* input_;
    Output* output_;

public:
    boost::signals2::signal<void()> display_request;
    boost::signals2::signal<void()> refresh_request;

private:
    connection_types::PointCloudMessage::ConstPtr message_;

    connection_types::CvMatMessage::Ptr result_;
};

}

#endif // CLOUD_RENDERER_H
