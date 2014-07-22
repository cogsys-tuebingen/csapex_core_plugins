#ifndef ROI_IMAGE_H
#define ROI_IMAGE_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_vision/roi_message.h>

namespace csapex
{

class ConnectorIn;

class ImageRoi : public InteractiveNode
{
    friend class ImageRoiAdapter;

public:
    ImageRoi();
    virtual ~ImageRoi();

    virtual QIcon getIcon() const;

    void setup();
    void setupParameters();
    void process();

    void setResult(connection_types::RoiMessage::Ptr result);

private:
    void submit();
    void drop();

protected:
    ConnectorIn*  input_;
    ConnectorOut* output_;
    cv::Size      last_mat_size_;

    connection_types::RoiMessage::Ptr result_;

public:
    boost::signals2::signal<void(QSharedPointer<QImage>)> display_request;
    boost::signals2::signal<void()>                       submit_request;
    boost::signals2::signal<void()>                       drop_request;
};

}

#endif // ROI_IMAGE_H
