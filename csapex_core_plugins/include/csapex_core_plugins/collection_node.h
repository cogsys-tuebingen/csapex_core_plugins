#ifndef COLLECTION_NODE_H
#define COLLECTION_NODE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>

/// SYSTEM
namespace csapex {

template <typename MessageType>
class CollectionNode : public Node
{
public:
    CollectionNode()
    {}

    virtual void setupParameters(Parameterizable &parameters) {
        addParameter(csapex::param::ParameterFactory::declareTrigger("process"), std::bind(&CollectionNode<MessageType>::doProcessCollection, this, std::ref(buffer_)));
        addParameter(csapex::param::ParameterFactory::declareTrigger("clear"), std::bind(&CollectionNode<MessageType>::clearCollection, this));
    }

    virtual void setup(NodeModifier& modifier) override
    {
        in_vector_generic  = modifier.addOptionalInput<connection_types::GenericVectorMessage, MessageType>("messages to collect");
        in_vector  = modifier.addOptionalInput<connection_types::VectorMessage, MessageType>("messages to collect");
        in_single  = modifier.addOptionalInput<MessageType>("message to collect");
    }

    virtual void process() override
    {
        if(msg::hasMessage(in_vector_generic)) {
            std::shared_ptr<std::vector<MessageType> const> input = msg::getMessage<connection_types::GenericVectorMessage, MessageType>(in_vector_generic);
            buffer_.insert(buffer_.end(), input->begin(), input->end());
        }
        if (msg::hasMessage(in_vector)) {
            connection_types::VectorMessage::ConstPtr input = msg::getMessage<connection_types::VectorMessage>(in_vector);
            for (auto&& msg : input->value) {
                if (auto casted = std::dynamic_pointer_cast<MessageType const>(msg))
                    buffer_.push_back(*casted);
            }
        }
        if(msg::hasMessage(in_single)) {
            std::shared_ptr<MessageType const> input = msg::getMessage<MessageType>(in_single);
            buffer_.push_back(*input);
        }
    }

protected:
    void doProcessCollection(std::vector<MessageType>& collection)
    {
        processCollection(collection);
    }

    virtual void processCollection(std::vector<MessageType>& collection) = 0;

    void clearCollection()
    {
        buffer_.clear();
    }

protected:
    Input* in_vector_generic;
    Input* in_vector;
    Input* in_single;

private:
    std::vector<MessageType> buffer_;
};

}

#endif // COLLECTION_NODE_H
