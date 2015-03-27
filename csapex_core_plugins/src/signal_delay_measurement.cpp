/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <ros/time.h>

using namespace csapex::connection_types;


namespace csapex
{

class SignalDelayMeasurement : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        start_ = std::chrono::system_clock::now();

        modifier.addSlot("start", [this]() {
            start_ = std::chrono::system_clock::now();
        });
        modifier.addSlot("stop", [this]() {
            end_ = std::chrono::system_clock::now();

            std::chrono::system_clock::duration delta = end_ - start_;

            ainfo << "Measured delay: " << delta.count() * 1e-6 << "ms" << std::endl;
        });
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start_;
    std::chrono::time_point<std::chrono::system_clock> end_;
};


}

CSAPEX_REGISTER_CLASS(csapex::SignalDelayMeasurement, csapex::Node)



