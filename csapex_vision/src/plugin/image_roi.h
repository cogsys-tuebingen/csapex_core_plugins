#ifndef ROI_IMAGE_H
#define ROI_IMAGE_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_vision/roi_message.h>

/// SYSTEM
#include <QImage>
#include <QSharedPointer>

namespace csapex
{

class Input;

class ImageRoi : public InteractiveNode
{
    friend class ImageRoiAdapter;

public:
    ImageRoi();
    virtual ~ImageRoi();

    void setup();
    void setupParameters();
    void process();

    void setResult(connection_types::RoiMessage::Ptr result);

private:
    void submit();
    void drop();

protected:
    Input*        input_;
    Output*       output_;
    cv::Size      last_mat_size_;

    connection_types::RoiMessage::Ptr result_;

public:
    boost::signals2::signal<void(QSharedPointer<QImage>)> display_request;
    boost::signals2::signal<void()>                       submit_request;
    boost::signals2::signal<void()>                       drop_request;
};

}

#endif // ROI_IMAGE_H
