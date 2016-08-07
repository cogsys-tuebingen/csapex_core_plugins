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
        out_vector_generic = modifier.addOutput<connection_types::GenericVectorMessage, MessageType>("messages processed");
    }

    virtual void process() override
    {
        std::shared_ptr<std::vector<MessageType> const> input_vector_generic;
        std::shared_ptr<std::vector<MessageType>> output_vector_generic;

        if(msg::hasMessage(in_vector_generic))
            input_vector_generic = msg::getMessage<connection_types::GenericVectorMessage, MessageType>(in_vector_generic);

        std::vector<MessageType *> access;
        if(msg::isConnected(out_vector_generic)) {
            output_vector_generic.reset(new std::vector<MessageType>);

            if(input_vector_generic) {
                for(const MessageType &msg : *input_vector_generic) {
                    output_vector_generic->emplace_back(msg);
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

    Output *out_vector_generic;

};

}

#endif // VECTORPROCESSNODE_H
