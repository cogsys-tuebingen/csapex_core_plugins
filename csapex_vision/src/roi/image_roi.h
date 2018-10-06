#ifndef ROI_IMAGE_H
#define ROI_IMAGE_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_opencv/roi_message.h>

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

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);

    virtual void beginProcess() override;
    virtual void finishProcess() override;

    void setResult(connection_types::RoiMessage::Ptr result);

private:
    void submit();
    void drop();

protected:
    Input* input_;
    Output* output_;
    cv::Size last_mat_size_;

    connection_types::RoiMessage::Ptr result_;

public:
    slim_signal::Signal<void(QImage)> display_request;
    slim_signal::Signal<void()> submit_request;
    slim_signal::Signal<void()> drop_request;
};

}  // namespace csapex

#endif  // ROI_IMAGE_H
