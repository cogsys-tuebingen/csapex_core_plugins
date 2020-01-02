#ifndef EXPORTROS_H
#define EXPORTROS_H

/// COMPONENT
#include <csapex_ros/ros_node.h>

/// SYSTEM
#include <boost/optional.hpp>

namespace csapex
{
class ExportRos : public RosNode
{
public:
    ExportRos();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupROS() override;
    void processROS() override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    void updateTopic();

private:
    Input* connector_;

    boost::optional<ros::Publisher> pub;

    std::string topic_;
    int queue_;

    int target_type_;
};

}  // namespace csapex
#endif  // EXPORTROS_H
