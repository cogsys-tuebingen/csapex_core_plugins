
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/token.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex_core_plugins/map_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN LookupMapEntry : public Node
{
    friend class LookupMapEntrySerializer;

public:
    LookupMapEntry()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_key_ = modifier.addInput<std::string>("Key");
        in_map_ = modifier.addOptionalInput<MapMessage>("Map");

        out_value_ = modifier.addOutput<AnyMessage>("Entry");

        modifier.addSlot<MapMessage>("Map", [this](const TokenConstPtr& token) {
            if (auto map = std::dynamic_pointer_cast<MapMessage const>(token->getTokenData())) {
                map_ = map;
            }
        });

        event_access_error_ = modifier.addEvent<std::string>("Access Error");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        if (msg::isConnected(in_map_)) {
            map_ = msg::getMessage<MapMessage>(in_map_);
        }

        std::string key = msg::getValue<std::string>(in_key_);

        if (map_) {
            for (KeyValueMessage part : map_->value) {
                apex_assert_msg(!part.value.first.empty(), "invalid key (empty)");
                apex_assert_msg(part.value.second, "invalid value (null)");

                if (part.value.first == key) {
                    msg::publish(out_value_, part.value.second);
                    return;
                }
            }
        } else {
            node_handle_->setWarning(std::string("No map received! Cannot lookup key ") + key);
        }

        std::shared_ptr<GenericValueMessage<std::string>> msg = std::make_shared<GenericValueMessage<std::string>>();
        msg->value = key;
        TokenPtr token = std::make_shared<Token>(msg);
        event_access_error_->triggerWith(token);
    }

private:
    Input* in_key_;
    Input* in_map_;

    Output* out_value_;

    Event* event_access_error_;

    MapMessage::ConstPtr map_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::LookupMapEntry, csapex::Node)
