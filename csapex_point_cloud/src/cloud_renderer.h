#ifndef CLOUD_RENDERER_H
#define CLOUD_RENDERER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <QMutex>
#include <QWaitCondition>

namespace csapex {

class CloudRenderer : public Node
{
    friend class CloudRendererAdapter;

public:
    CloudRenderer();

    virtual void setup();
    virtual void process();

    virtual void stop();

    void publishImage(const cv::Mat &img);

private:
    void refresh();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

public:
    boost::signals2::signal<void()> display_request;
    boost::signals2::signal<void()> refresh_request;

private:
    connection_types::PointCloudMessage::Ptr message_;

    connection_types::CvMatMessage::Ptr result_;

    QMutex result_mutex_;
    QWaitCondition wait_for_view_;

    bool stopped_;
};

}

#endif // CLOUD_RENDERER_H
