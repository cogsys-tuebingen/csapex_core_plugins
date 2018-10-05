#ifndef ROS_PARAM_H
#define ROS_PARAM_H

/// COMPONENT

/// PROJECT
#include <csapex_ros/ros_node.h>

/// SYSTEM
#include <ros/service.h>

namespace csapex
{
class RosParam : public csapex::RosNode
{
public:
    RosParam();

    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void setupROS() override;
    virtual void processROS() override;

private:
    void update();
};

}  // namespace csapex

#endif  // ROS_PARAM_H
