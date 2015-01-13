#ifndef COLLECTION_NODE_H
#define COLLECTION_NODE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/signal/trigger.h>

/// SYSTEM
namespace csapex {

template <typename MessageType>
class CollectionNode : public Node
{
public:
    CollectionNode()
    {}

    void setupParameters()
    {
        addParameter(param::ParameterFactory::declareTrigger("process"), boost::bind(&CollectionNode<MessageType>::doProcessCollection, this, boost::ref(buffer_)));
        addParameter(param::ParameterFactory::declareTrigger("clear"), boost::bind(&CollectionNode<MessageType>::clearCollection, this));
    }

    void setup()
    {
        in_vector  = modifier_->addOptionalInput<connection_types::GenericVectorMessage, MessageType>("messages to collect");
        in_single  = modifier_->addOptionalInput<MessageType>("message to collect");
    }

    void process()
    {
        if(in_vector->hasMessage()) {
            std::shared_ptr<std::vector<MessageType> const> input = in_vector->getMessage<connection_types::GenericVectorMessage, MessageType>();
            buffer_.insert(buffer_.end(), input->begin(), input->end());
        }
        if(in_single->hasMessage()) {
            std::shared_ptr<MessageType const> input = in_single->getMessage<MessageType>();
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
    Input* in_vector;
    Input* in_single;

private:
    std::vector<MessageType> buffer_;
};

}

#endif // COLLECTION_NODE_H
