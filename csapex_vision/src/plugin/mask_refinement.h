#ifndef MASK_REFINEMENT_H
#define MASK_REFINEMENT_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_vision/cv_mat_message.h>

class QImage;

namespace csapex
{

class MaskRefinement : public InteractiveNode
{
public:
    MaskRefinement();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

    void setMask(const QImage &m);

protected:
    virtual void beginProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters) override;
    virtual void finishProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters) override;

private:
    void ok();
    void drop();

public:
    csapex::slim_signal::Signal<void(QImage&, QImage&)> input;
    csapex::slim_signal::Signal<void()> next_image;

    csapex::slim_signal::Signal<void()> update_brush;

private:
    Input* in_mask_;
    Input* in_img_;

    Output* out_;

    cv::Mat mask_;

    bool has_img_;
    cv::Mat img_;

    connection_types::CvMatMessage::Ptr result_;
};

} /// NAMESPACE

#endif // MASK_REFINEMENT_H
