#ifndef EXPORTROS_H
#define EXPORTROS_H

/// COMPONENT
#include <csapex_ros/ros_node.h>

namespace csapex {

class ExportRos : public RosNode
{
public:
    ExportRos();

    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupROS() override;
    virtual void processROS() override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    void updateTopic();

private:
    Input* connector_;

    bool create_pub;

    ros::Publisher pub;

    std::string topic_;
    int queue_;
};

}
#endif // EXPORTROS_H
