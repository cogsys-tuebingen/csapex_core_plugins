#ifndef ASSIGN_ROI_CLASS_H
#define ASSIGN_ROI_CLASS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_vision/roi_message.h>

/// SYSTEM
#include <QImage>
#include <QSharedPointer>

namespace vision_plugins {
class AssignROIClass : public csapex::InteractiveNode
{
    friend class AssignROIClassAdapter;

public:
    AssignROIClass();
    virtual ~AssignROIClass();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);

    virtual void beginProcess() override;
    virtual void finishProcess() override;

    void setActiveClassColor(const int r, const int g, const int b);

private:
    void submit();
    void drop();
    void clear();
    void setColor();
    void setClass();
    void display();

protected:
    csapex::Input*    in_image_;
    csapex::Input*    in_rois_;
    csapex::Output*   out_rois_;

    cv::Mat                                                image_;
    std::vector<csapex::connection_types::RoiMessage>      rois_;

public:
    csapex::slim_signal::Signal<void(QImage)> display_request;
    csapex::slim_signal::Signal<void()>                       submit_request;
    csapex::slim_signal::Signal<void()>                       drop_request;
    csapex::slim_signal::Signal<void()>                       clear_request;
    csapex::slim_signal::Signal<void(int)>                    set_class;
    csapex::slim_signal::Signal<void(int,int,int)>            set_color;
};

}

#endif // ASSIGN_ROI_CLASS_H
