#include <csapex_testing/stepping_test.h>

#include <csapex/core/csapex_core.h>
#include <csapex/core/graphio.h>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/io.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/token.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/plugin/plugin_locator.h>

#include "../../src/io/file_importer.h"
#include <csapex_core_plugins/register_core_plugins.h>

using namespace csapex;
using namespace connection_types;

namespace csapex
{
class TestPluginLocator : public PluginLocator
{
public:
    TestPluginLocator(Settings& settings) : PluginLocator(settings)
    {
#ifdef PACKAGE_XML
        registerLocator<CorePlugin>([](std::vector<std::string>& paths) { paths.push_back(std::string(PACKAGE_XML)); });
        registerLocator<Node>([](std::vector<std::string>& paths) { paths.push_back(std::string(PACKAGE_XML)); });
#else
        std::cerr << "Cannot find plugins for this test!" << std::endl;
#endif
    }
};

class FileImporterTest : public NodeConstructingTest
{
public:
    FileImporterTest()
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

TEST_F(FileImporterTest, FileImporterConstruction)
{
    NodeFacadeImplementationPtr importer = factory.makeNode("csapex::FileImporter", graph->generateUUID("importer"), graph);

    ASSERT_NE(nullptr, importer);
}

TEST_F(FileImporterTest, ImportDirectoryCanBeEnabled)
{
    NodeFacadeImplementationPtr importer = factory.makeNode("csapex::FileImporter", graph->generateUUID("importer"), graph);
    ASSERT_NE(nullptr, importer);

    NodeFacadeImplementationPtr sink = factory.makeNode("AnySink", graph->generateUUID("sink"), graph);
    ASSERT_NE(nullptr, sink);

    importer->setParameter("import directory", true);
    importer->setParameter("directory", std::string(TEST_RES_DIR) + "/input_sequence");

    ASSERT_TRUE(importer->getNode()->hasChangedParameters());
    importer->handleChangedParameters();
    ASSERT_TRUE(importer->canProcess());

    NodePtr importer_node = importer->getNode();
    ASSERT_TRUE(importer_node->canProcess());
}

TEST_F(FileImporterTest, ImportDirectory)
{
    NodeFacadeImplementationPtr importer_facade = factory.makeNode("csapex::FileImporter", graph->generateUUID("importer"), graph);
    ASSERT_NE(nullptr, importer_facade);

    NodePtr importer_node = importer_facade->getNode();
    ASSERT_NE(nullptr, importer_node);

    std::shared_ptr<FileImporter> importer = std::dynamic_pointer_cast<FileImporter>(importer_node);
    ASSERT_NE(nullptr, importer);

    NodeFacadeImplementationPtr sink = factory.makeNode("AnySink", graph->generateUUID("sink"), graph);
    ASSERT_NE(nullptr, sink);

    importer_facade->setParameter("import directory", true);
    importer_facade->setParameter("directory", std::string(TEST_RES_DIR) + "/input_sequence");
    importer_facade->handleChangedParameters();

    importer->import();

    InputPtr input = csapex::testing::getInput(sink, "in_0");
    EXPECT_NE(nullptr, input);

    OutputPtr output = csapex::testing::getOutput(importer_facade, "out_0");
    EXPECT_NE(nullptr, output);

    ConnectionPtr connection = DirectConnection::connect(output, input);
    EXPECT_NE(nullptr, connection);

    importer_node->process(*importer->getNodeHandle(), *importer_node, [&](ProcessingFunction fn) {
        auto value_msg = testing::getAddedMessage<std::string>(output);
        ASSERT_NE(nullptr, value_msg);
        ASSERT_EQ("1", value_msg->value);

        importer_node->process(*importer->getNodeHandle(), *importer_node, [&](ProcessingFunction fn) {
            auto value_msg = testing::getAddedMessage<std::string>(output);
            ASSERT_NE(nullptr, value_msg);
            ASSERT_EQ("2", value_msg->value);

            importer_node->process(*importer->getNodeHandle(), *importer_node, [&](ProcessingFunction fn) {
                auto value_msg = testing::getAddedMessage<std::string>(output);
                ASSERT_NE(nullptr, value_msg);
                ASSERT_EQ("3", value_msg->value);
            });
        });
    });
}

TEST_F(FileImporterTest, StoreAndReloadKeepsConnections)
{
    YAML::Node store;

    UUID importer_id;

    {
        auto graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphImplementation>());
        auto graph = graph_node->getLocalGraph();
        GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

        // importer
        importer_id = graph->generateUUID("importer");

        NodeFacadeImplementationPtr importer_facade = factory.makeNode("csapex::FileImporter", importer_id, graph);
        ASSERT_NE(nullptr, importer_facade);
        main_graph_facade.addNode(importer_facade);

        NodePtr importer_node = importer_facade->getNode();
        ASSERT_NE(nullptr, importer_node);

        std::shared_ptr<FileImporter> importer = std::dynamic_pointer_cast<FileImporter>(importer_node);
        ASSERT_NE(nullptr, importer);

        importer_facade->setParameter("import directory", true);
        importer_facade->setParameter("directory", std::string(TEST_RES_DIR) + "/input_sequence");
        ASSERT_TRUE(importer_node->hasChangedParameters());
        importer_facade->handleChangedParameters();
        ASSERT_TRUE(importer->canProcess());

        importer->import();

        OutputPtr output = testing::getOutput(importer_facade, "out_0");
        ASSERT_NE(nullptr, output);

        // sinks
        NodeFacadeImplementationPtr sink1 = factory.makeNode("AnySink", graph->generateUUID("sink"), graph);
        ASSERT_NE(nullptr, sink1);
        main_graph_facade.addNode(sink1);
        InputPtr input1 = testing::getInput(sink1, "in_0");
        ASSERT_NE(nullptr, input1);
        main_graph_facade.connect(importer_facade, 0, sink1, 0);

        NodeFacadeImplementationPtr sink2 = factory.makeNode("AnySink", graph->generateUUID("sink"), graph);
        ASSERT_NE(nullptr, sink2);
        main_graph_facade.addNode(sink2);
        InputPtr input2 = testing::getInput(sink2, "in_0");
        ASSERT_NE(nullptr, input2);
        main_graph_facade.connect(importer_facade, 0, sink2, 0);

        GraphIO io(main_graph_facade, &factory, true);
        ASSERT_NO_THROW(io.saveGraphTo(store));
    }

    {
        auto graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphImplementation>());
        auto graph = graph_node->getLocalGraph();
        GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

        GraphIO io(main_graph_facade, &factory, true);
        ASSERT_NO_THROW(io.loadGraphFrom(store));

        NodeFacadeImplementationPtr node = testing::getNodeFacade(main_graph_facade, importer_id);
        ASSERT_NE(nullptr, node);

        OutputPtr out = testing::getOutput(node, "out_0");
        ASSERT_NE(nullptr, out);

        ASSERT_EQ(2u, out->getConnections().size());
    }
}

}  // namespace csapex
