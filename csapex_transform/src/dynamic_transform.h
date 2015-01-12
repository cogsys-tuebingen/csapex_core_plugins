#ifndef DYNAMIC_TRANSFORM_H
#define DYNAMIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/node.h>
#include <utils_param/set_parameter.h>

/// SYSTEM
#include <ros/time.h>

namespace csapex {

class DynamicTransform : public csapex::Node
{
public:
    DynamicTransform();

    virtual void setup();
    virtual void setupParameters();

    virtual void process();
    bool canTick();
    virtual void tick();
    void update();

    void refresh();
    void resetTf();

private:
    void publishTransform(const ros::Time& time);

private:
    Output* output_;
    Output* output_frame_;

    Input* frame_in_source_;
    Input* frame_in_target_;
    Input* time_in_;

    Trigger* reset_;

    bool init_;
    int initial_retries_;

    param::SetParameter::Ptr source_p;
    param::SetParameter::Ptr target_p;
};

}

#endif // DYNAMIC_TRANSFORM_H
