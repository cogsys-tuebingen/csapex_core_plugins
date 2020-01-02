#ifndef ASSIGN_ROI_CLASS_H
#define ASSIGN_ROI_CLASS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_opencv/roi_message.h>

/// SYSTEM
#include <QImage>
#include <QSharedPointer>

namespace csapex
{
class LabelROIs : public csapex::InteractiveNode
{
    friend class LabelROIsAdapter;

public:
    LabelROIs();
    virtual ~LabelROIs();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

    void beginProcess() override;
    void finishProcess() override;

    void setActiveClassColor(const int r, const int g, const int b);

private:
    void submit();
    void drop();
    void clear();
    void setColor();
    void setClass();
    void display();

protected:
    csapex::Input* in_image_;
    csapex::Input* in_rois_;
    csapex::Output* out_rois_;

    cv::Mat image_;
    std::vector<csapex::connection_types::RoiMessage> rois_;

public:
    slim_signal::Signal<void(QImage)> display_request;
    slim_signal::Signal<void()> submit_request;
    slim_signal::Signal<void()> drop_request;
    slim_signal::Signal<void()> clear_request;
    slim_signal::Signal<void(int)> set_class;
    slim_signal::Signal<void(int, int, int)> set_color;
};

}  // namespace csapex

#endif  // ASSIGN_ROI_CLASS_H
