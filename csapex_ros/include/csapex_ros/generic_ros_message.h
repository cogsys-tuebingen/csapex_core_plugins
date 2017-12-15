#ifndef GENERIC_ROS_MESSAGE_H
#define GENERIC_ROS_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/utility/type.h>

/// SYSTEM
#include <ros/message_traits.h>
#include <csapex/utility/suppress_warnings_start.h>
    #include <topic_tools/shape_shifter.h>
#include <csapex/utility/suppress_warnings_end.h>

namespace csapex {

namespace connection_types {


class GenericRosMessage : public MessageTemplate<std::shared_ptr<topic_tools::ShapeShifter const>, GenericRosMessage>
{
public:
    GenericRosMessage();

    bool canConnectTo(const TokenData *other_side) const override
    {
        return true;
    }

    bool acceptsConnectionFrom(const TokenData* other_side) const override
    {
        return true;
    }
};


/// TRAITS
template <>
struct type<GenericRosMessage> {
    static std::string name() {
        return "GenericRosMessage";
    }
};

}

namespace msg
{

template <typename R, typename S>
struct MessageCaster<connection_types::GenericPointerMessage<R>, S, void>
{
    static std::shared_ptr<connection_types::GenericPointerMessage<R> const> constcast(const std::shared_ptr<S const>& msg)
    {
        auto rosmsg = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(msg);
        if(rosmsg) {
            try {
                auto boost_ptr = rosmsg->value->template instantiate<R>();
                auto msg = connection_types::makeEmpty<connection_types::GenericPointerMessage<R>>();
                msg->value = shared_ptr_tools::to_std_shared(boost_ptr);
                return msg;
            } catch(const ros::Exception& e) {
                // no success...
            }
        }
        return MessageCaster<connection_types::GenericPointerMessage<R>, S, void>::constcast(msg);
    }
    static std::shared_ptr<connection_types::GenericPointerMessage<R>> cast(const std::shared_ptr<S>& msg)
    {
        auto rosmsg = std::dynamic_pointer_cast<connection_types::GenericRosMessage>(msg);
        if(rosmsg) {
            try {
                auto boost_ptr = rosmsg->value->template instantiate<R>();
                auto msg = connection_types::makeEmpty<connection_types::GenericPointerMessage<R>>();
                msg->value = shared_ptr_tools::to_std_shared(boost_ptr);
                return msg;
            } catch(const ros::Exception& e) {
                // no success...
            }
        }
        return MessageCaster<connection_types::GenericPointerMessage<R>, S, void>::cast(msg);
    }
};

}


}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::GenericRosMessage> {
  static Node encode(const csapex::connection_types::GenericRosMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::GenericRosMessage& rhs);
};
}

#endif // GENERIC_ROS_MESSAGE_H

