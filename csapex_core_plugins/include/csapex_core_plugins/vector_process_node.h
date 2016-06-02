#ifndef VECTORPROCESSNODE_H
#define VECTORPROCESSNODE_H


/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>

/// SYSTEM
namespace csapex {

template <typename MessageType>
class VectorProcessNode : public Node
{
public:
    VectorProcessNode()
    {}

    virtual void setupParameters(Parameterizable &parameters) {
    }

    virtual void setup(NodeModifier& modifier) override
    {
        in_vector_generic  = modifier.addOptionalInput<connection_types::GenericVectorMessage, MessageType>("messages to process");
        in_vector  = modifier.addOptionalInput<connection_types::VectorMessage, MessageType>("messages to process");
        out_vector_generic = modifier.addOutput<connection_types::GenericVectorMessage, MessageType>("messages processed");
        out_vector = modifier.addOutput<connection_types::VectorMessage, MessageType>("messages processed");
    }

    virtual void process() override
    {
        std::shared_ptr<std::vector<MessageType> const> input_vector_generic;
        connection_types::VectorMessage::ConstPtr input_vector;
        std::shared_ptr<std::vector<MessageType>> output_vector_generic;
        connection_types::VectorMessage::Ptr      output_vector;

        if(msg::hasMessage(in_vector_generic))
            input_vector_generic = msg::getMessage<connection_types::GenericVectorMessage, MessageType>(in_vector_generic);
        if(msg::hasMessage(in_vector))
            input_vector = msg::getMessage<connection_types::VectorMessage>(in_vector);

        std::vector<MessageType *> access;
        if(msg::isConnected(out_vector)) {
            output_vector = connection_types::VectorMessage::make<MessageType>();
            if(input_vector_generic) {
                for(const MessageType &msg : *input_vector_generic) {
                    output_vector->value.push_back(msg.clone());
                    access.push_back((MessageType *) output_vector->value.back().get());
                }
            }
            if(input_vector) {
                for(auto msg : input_vector->value) {
                    output_vector->value.push_back(msg->cloneAs<MessageType>());
                    access.push_back((MessageType *) output_vector->value.back().get());
                }
            }

            doProcessCollection(access);

            if(msg::isConnected(out_vector_generic)) {
                output_vector_generic.reset(new std::vector<MessageType>);
                for(auto msg : access) {
                    output_vector_generic->push_back(*msg);
                }
                msg::publish(out_vector, output_vector);
            }
            msg::publish(out_vector, output_vector);

        } else if(msg::isConnected(out_vector_generic)) {
            output_vector_generic.reset(new std::vector<MessageType>);

            if(input_vector_generic) {
                for(const MessageType &msg : *input_vector_generic) {
                    output_vector_generic->emplace_back(msg);
                }
            }
            if(input_vector) {
                for (auto&& msg : input_vector->value) {
                    if (auto casted = std::dynamic_pointer_cast<MessageType const>(msg))
                        output_vector_generic->emplace_back(*casted);
                }
            }

            for(MessageType &msg : *output_vector_generic)
                access.push_back(&msg);

            doProcessCollection(access);
            msg::publish<connection_types::GenericVectorMessage, MessageType>(out_vector_generic, output_vector_generic);
        }
    }

protected:
    void doProcessCollection(std::vector<MessageType*>& collection)
    {
        processCollection(collection);
    }

    virtual void processCollection(std::vector<MessageType*>& collection) = 0;

protected:
    Input* in_vector_generic;
    Input* in_vector;

    Output *out_vector_generic;
    Output *out_vector;

};

}

#endif // VECTORPROCESSNODE_H
