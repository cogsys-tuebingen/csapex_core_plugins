#ifndef DYNAMIC_TRANSFORM_H
#define DYNAMIC_TRANSFORM_H

/// PROJECT
#include <csapex/param/set_parameter.h>
#include <csapex_ros/ros_node.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <boost/optional.hpp>
#include <ros/time.h>
#include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

namespace csapex
{
class DynamicTransform : public csapex::RosNode
{
public:
    DynamicTransform();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupROS() override;
    void setupParameters(Parameterizable& parameters) override;

    void processROS() override;

    void update();

    void refresh();
    void resetTf();

    void freeze(bool frozen);

private:
    void publishTransform(const ros::Time& time);

private:
    Output* output_;

    Input* time_in_;

    Event* reset_;

    bool init_;
    int initial_retries_;

    param::SetParameter::Ptr source_p;
    param::SetParameter::Ptr target_p;

    bool exact_time_;

    bool frozen_;

    double roll;
    double pitch;
    double yaw;
    double x;
    double y;
    double z;

    boost::optional<tf::Transform> last_transform;
};

}  // namespace csapex

#endif  // DYNAMIC_TRANSFORM_H
