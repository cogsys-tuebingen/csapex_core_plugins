/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/input.h>
#include <csapex/msg/io.h>
#include <csapex/msg/marker_message.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/slot.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/ros_message_conversion.h>

/// SYSTEM
#include <rosbag/bag.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN BagWriter : public Node, public VariadicInputs, public VariadicSlots
{
public:
    BagWriter() : is_open_(false), compress_(false)
    {
    }

    ~BagWriter()
    {
        stop();
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        setupVariadic(node_modifier);
    }

    void setupVariadicParameters(Parameterizable& parameters) override
    {
        VariadicInputs::setupVariadicParameters(parameters);
        VariadicSlots::setupVariadicParameters(parameters);
    }

    void setupParameters(Parameterizable& parameters) override
    {
        setupVariadicParameters(parameters);

        parameters.addParameter(param::factory::declareFileOutputPath("file", "/tmp/bag.bag", "*.bag"), [this](param::Parameter* p) {
            std::string file_name = p->as<std::string>();
            if (file_name == file_name_) {
                return;
            }
            file_name_ = file_name;
            start();
        });

        parameters.addParameter(param::factory::declareTrigger("reset"), [this](param::Parameter* p) { start(); });

        parameters.addParameter(param::factory::declareTrigger("save and close"), [this](param::Parameter* p) { stop(); });

        parameters.addParameter(param::factory::declareBool("use_lz4_compression", false),
                                [this](param::Parameter* p) {
            bool compress = p->as<bool>();
            if( compress == compress_){
                return;
            }
            compress_ = compress;
            start();
        });
    }

    bool canProcess() const override
    {
        return !node_modifier_->getMessageInputs().empty();
    }

    void process() override
    {
        if (!is_open_) {
            node_handle_->setWarning("bag file is closed, cannot write");
            return;
        } else {
            node_handle_->setNoError();
        }

        std::vector<InputPtr> inputs = node_modifier_->getMessageInputs();
        for (std::size_t i = 0; i < inputs.size(); i++) {
            Input* in = inputs[i].get();
            if (msg::hasMessage(in)) {
                connection_types::MessageConstPtr message = msg::getMessage<connection_types::Message>(in);

                writeMessage(message, in->getLabel());
            }
        }
    }

    void writeMessage(const connection_types::MessageConstPtr& message, const std::string& topic)
    {
        auto pos = topic_to_type_.find(topic);

        connection_types::GenericVectorMessage::ConstPtr vector = std::dynamic_pointer_cast<connection_types::GenericVectorMessage const>(message);

        TokenData::ConstPtr type;
        if (vector) {
            type = vector->nestedType();
        } else {
            type = message;
        }

        int selected_target_type = -1;

        if (pos != topic_to_type_.end()) {
            selected_target_type = pos->second;

        } else {
            RosMessageConversion& ros_conv = RosMessageConversion::instance();

            std::map<std::string, int> possible_conversions = ros_conv.getAvailableRosConversions(type);
            if (possible_conversions.size() != 1) {
                param::ParameterPtr p;
                std::string param_name = topic + "_target_type";
                if (!hasParameter(param_name)) {
                    p = param::factory::declareParameterSet(param_name, possible_conversions, 0);
                    addTemporaryParameter(p);
                    addParameterCallback(p, [this, topic](param::Parameter* p) { topic_to_type_[topic] = p->as<int>(); });
                } else {
                    p = getParameter(param_name);
                }

                topic_to_type_[topic] = p->as<int>();
                selected_target_type = topic_to_type_[topic];
            }
        }

        if (vector) {
            for (std::size_t i = 0, n = vector->nestedValueCount(); i < n; ++i) {
                connection_types::MessageConstPtr msg = std::dynamic_pointer_cast<connection_types::Message const>(vector->nestedValue(i));
                if (msg) {
                    RosMessageConversion::instance().write(bag, msg, topic, selected_target_type);
                }
            }
        } else {
            RosMessageConversion::instance().write(bag, message, topic, selected_target_type);
        }
    }

    void stop()
    {
        if (is_open_) {
            bag.close();
            is_open_ = false;
        }
    }

    void start()
    {
        try {
            if (is_open_) {
                stop();
            }
            bag.open(file_name_, rosbag::bagmode::Write);
            if(compress_){
                bag.setCompression(rosbag::CompressionType::LZ4);
            } else {
                bag.setCompression(rosbag::CompressionType::Uncompressed);
            }
            is_open_ = true;
        } catch (const rosbag::BagException& e) {
            is_open_ = false;
            node_handle_->setError(e.what());
        }
    }

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        Input* result = VariadicInputs::createVariadicInput(makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);

        return result;
    }

    virtual csapex::Slot* createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void(const TokenPtr&)> callback, bool active, bool blocking) override
    {
        auto cb = [this](Slot* slot, const TokenPtr& token) {
            auto message = std::dynamic_pointer_cast<connection_types::Message const>(token->getTokenData());
            auto marker = std::dynamic_pointer_cast<connection_types::MarkerMessage const>(token->getTokenData());
            if (marker == nullptr) {
                writeMessage(message, slot->getLabel());
            }
        };

        return VariadicSlots::createVariadicSlot(makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, cb, active, blocking);
    }

    void write(const std::string& label, long stamp, std::shared_ptr<topic_tools::ShapeShifter const> instance)
    {
        if (instance) {
            apex_assert(!instance->getMD5Sum().empty());
            if (stamp > 0) {
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
                return createVariadicSlot(type, label, [](const TokenPtr&) {}, false, true);
            default:
                throw std::logic_error(std::string("Variadic port of type ") + port_type::name(port_type) + " is not supported.");
        }
    }

private:
    rosbag::Bag bag;
    bool is_open_;
    bool compress_;
    std::string file_name_;

    std::map<std::string, int> topic_to_type_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::BagWriter, csapex::Node)
