/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/slot.h>
#include <csapex/msg/input.h>
#include <csapex/msg/message.h>
#include <csapex_ros/ros_message_conversion.h>

/// SYSTEM
#include <rosbag/bag.h>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN BagWriter : public Node, public VariadicInputs, public VariadicSlots
{
public:
    BagWriter()
    {}

    ~BagWriter()
    {
        stop();
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        setupVariadic(node_modifier);

        node_modifier.addSlot("save", [this](){
            bag.close();
        });
    }

    void setupVariadicParameters(Parameterizable &parameters)
    {
        VariadicInputs::setupVariadicParameters(parameters);
        VariadicSlots::setupVariadicParameters(parameters);
    }

    void setupParameters(Parameterizable& parameters)
    {
        setupVariadicParameters(parameters);

        parameters.addParameter(param::ParameterFactory::declareFileOutputPath("file", "/tmp/bag.bag", "*.bag"), [this](param::Parameter* p) {
            bag.open(p->as<std::string>(), rosbag::bagmode::Write);
        });

        parameters.addParameter(param::ParameterFactory::declareTrigger("reset"), [this](param::Parameter* p) {
            bag.close();
            bag.open(readParameter<std::string>("file"), rosbag::bagmode::Write);
        });

        parameters.addParameter(param::ParameterFactory::declareTrigger("close"), [this](param::Parameter* p) {
            stop();
        });
    }
    void process()
    {
        std::vector<InputPtr> inputs = node_modifier_->getMessageInputs();
        for(std::size_t i = 0 ; i < inputs.size() ; i++) {
            Input *in = inputs[i].get();
            if(msg::hasMessage(in)) {
                connection_types::MessageConstPtr message = msg::getMessage<connection_types::Message>(in);

                RosMessageConversion::instance().write(bag, message, in->getLabel());
            }
        }
    }

    void stop()
    {
        bag.close();
    }

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
    }

    virtual csapex::Slot* createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void (const TokenPtr&)> callback, bool active, bool asynchronous) override
    {
        auto cb = [this](Slot* slot, const TokenPtr& token) {
            auto message = std::dynamic_pointer_cast<connection_types::Message const>(token->getTokenData());
            RosMessageConversion::instance().write(bag, message, slot->getLabel());
        };

        return VariadicSlots::createVariadicSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, cb, active, asynchronous);
    }

    void write(const std::string& label, long stamp, std::shared_ptr<topic_tools::ShapeShifter const> instance)
    {
        if(instance) {
            apex_assert(!instance->getMD5Sum().empty());
            if(stamp > 0) {
                ros::Time time;
                time.fromNSec(stamp * 1e3);
                bag.write(label, time, *instance);
            }
        } else {
            aerr << "cannot write " << label << ", instance is null" << std::endl;
        }
    }


    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        apex_assert_hard(variadic_modifier_);
        switch (port_type) {
        case ConnectorType::INPUT:
            return createVariadicInput(type, label, optional);
        case ConnectorType::SLOT_T:
            return createVariadicSlot(type, label, [](const TokenPtr&){}, false, false);
        default:
            throw std::logic_error(std::string("Variadic port of type ") + port_type::name(port_type) + " is not supported.");
        }
    }

private:
    Output* output;

    rosbag::Bag bag;
};

}

CSAPEX_REGISTER_CLASS(csapex::BagWriter, csapex::Node)
