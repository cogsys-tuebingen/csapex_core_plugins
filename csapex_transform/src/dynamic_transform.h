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
    virtual void tick();
    void update();

    void refresh();
    void resetTf();

private:
    void publishTransform(const ros::Time& time);

private:
    Output* output_;
    Output* output_frame_;

    Input* frame_in_from_;
    Input* frame_in_to_;
    Input* time_in_;

    param::SetParameter::Ptr from_p;
    param::SetParameter::Ptr to_p;
};

}

#endif // DYNAMIC_TRANSFORM_H
