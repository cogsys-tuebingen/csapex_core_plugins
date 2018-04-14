#include <csapex_testing/stepping_test.h>

#include <csapex/model/graph/graph_impl.h>
#include <csapex/core/graphio.h>
#include <csapex/core/csapex_core.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/model/node_handle.h>
#include <csapex/msg/output.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/token.h>
#include <csapex/model/io.h>
#include <csapex/msg/direct_connection.h>
#include <csapex_core_plugins/composite_message.h>
#include <csapex_core_plugins/duration_message.h>
#include <csapex_core_plugins/timestamp_message.h>

#include <csapex_core_plugins/register_core_plugins.h>

using namespace csapex;
using namespace connection_types;

namespace csapex {

class TestPluginLocator : public PluginLocator
{
public:
    TestPluginLocator(Settings& settings)
        : PluginLocator(settings)
    {
#ifdef PACKAGE_XML
        registerLocator<CorePlugin>([this](std::vector<std::string>& paths){
            paths.push_back(std::string(PACKAGE_XML));
        });
        registerLocator<Node>([this](std::vector<std::string>& paths){
            paths.push_back(std::string(PACKAGE_XML));
        });
#else
        std::cerr << "Cannot find plugins for this test!" << std::endl;
#endif
    }
};

class CorePluginsSerializationTest : public NodeConstructingTest
{
public:
    CorePluginsSerializationTest()
    {
        locator = std::make_shared<TestPluginLocator>(settings);
        core = std::make_shared<CsApexCore>(settings, eh, locator, node_factory, nullptr);
        node_factory->setPluginLocator(locator.get());
        core->init();
    }

private:
    SettingsImplementation settings;
    PluginLocatorPtr locator;

    TestExceptionHandler eh;
    csapex::CsApexCorePtr core;
};

TEST_F(CorePluginsSerializationTest, CompositeMessageSerialization)
{
    SerializationBuffer data;
    {
        CompositeMessage::Ptr message = std::make_shared<CompositeMessage>();

        message->value.push_back(std::make_shared<TimestampMessage>(TimestampMessage::Tp(std::chrono::microseconds(23))));
        message->value.push_back(std::make_shared<DurationMessage>(std::chrono::microseconds(42)));

        message->serialize(data);
    }

    {
        CompositeMessage::Ptr message = std::make_shared<CompositeMessage>();
        message->deserialize(data);

        ASSERT_EQ(2, message->value.size());

        std::shared_ptr<TimestampMessage const> ts = std::dynamic_pointer_cast<TimestampMessage const>(message->value.at(0));
        std::shared_ptr<DurationMessage const> dur = std::dynamic_pointer_cast<DurationMessage const>(message->value.at(1));

        ASSERT_NE(nullptr, ts);
        ASSERT_NE(nullptr, dur);

        ASSERT_EQ(23, ts->value.time_since_epoch().count());
        ASSERT_EQ(42, dur->value.count());
    }
}


}
