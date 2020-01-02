#ifndef COLLECTION_NODE_H
#define COLLECTION_NODE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex_core_plugins/csapex_core_lib_export.h>

/// SYSTEM
namespace csapex
{
template <typename MessageType, typename Allocator = std::allocator<MessageType>>
class CollectionNode : public Node
{
public:
    CollectionNode()
    {
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(csapex::param::factory::declareTrigger("process"), std::bind(&CollectionNode<MessageType, Allocator>::doProcessCollection, this, std::ref(buffer_)));
        parameters.addParameter(csapex::param::factory::declareTrigger("clear"), std::bind(&CollectionNode<MessageType, Allocator>::clearCollection, this));
    }

    virtual void setup(NodeModifier& modifier) override
    {
        in_vector_generic = modifier.addOptionalInput<connection_types::GenericVectorMessage, MessageType>("messages to collect");
        in_vector = modifier.addOptionalInput<connection_types::AnyMessage>("messages to collect (deprecated)");
        in_single = modifier.addOptionalInput<MessageType>("message to collect");
        event_processed = modifier.addEvent("Processed Collection");
    }

    virtual void process() override
    {
        if (msg::hasMessage(in_vector_generic)) {
            std::shared_ptr<std::vector<MessageType> const> input = msg::getMessage<connection_types::GenericVectorMessage, MessageType>(in_vector_generic);
            buffer_.insert(buffer_.end(), input->begin(), input->end());
        }
        if (msg::hasMessage(in_vector)) {
            throw std::runtime_error("VectorMessage no longer exists");
        }
        if (msg::hasMessage(in_single)) {
            std::shared_ptr<MessageType const> input = msg::getMessage<MessageType>(in_single);
            buffer_.push_back(*input);
        }
    }

protected:
    void doProcessCollection(std::vector<MessageType, Allocator>& collection)
    {
        if (collection.empty()) {
            node_modifier_->setError("Collection is empty!");
            return;
        } else {
            node_modifier_->setNoError();
        }

        if (processCollection(collection)) {
            event_processed->trigger();
        } else {
            node_modifier_->setError("Could not process the collection!");
        }
    }

    virtual bool processCollection(std::vector<MessageType, Allocator>& collection) = 0;

    void clearCollection()
    {
        buffer_.clear();
    }

protected:
    Input* in_vector_generic;
    Input* in_vector;
    Input* in_single;
    Event* event_processed;

private:
    std::vector<MessageType, Allocator> buffer_;
};

}  // namespace csapex

#endif  // COLLECTION_NODE_H
