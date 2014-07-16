#ifndef SCAN_LABELER_H
#define SCAN_LABELER_H

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

protected:
    ConnectorIn*  input_;
    ConnectorOut* output_;

    connection_types::RoiMessage::Ptr result_;

public:
    boost::signals2::signal<void(cv::Mat* )> display_request;
    boost::signals2::signal<void()>          submit_request;
};

}

#endif // SCAN_LABELER_H
