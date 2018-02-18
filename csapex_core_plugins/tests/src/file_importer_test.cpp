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
#include <csapex/msg/direct_connection.h>

#include <csapex_core_plugins/register_core_plugins.h>

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

TEST_F(FileImporterTest, ImportDirectory)
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

    InputPtr input = sink->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(sink->getUUID(), "in_0"));
    ASSERT_NE(nullptr, input);

    OutputPtr output = importer->getNodeHandle()->getOutput(UUIDProvider::makeDerivedUUID_forced(importer->getUUID(), "out_0"));
    ASSERT_NE(nullptr, output);

    ConnectionPtr c = DirectConnection::connect(output, input);

    importer_node->process(*importer->getNodeHandle(), *importer_node, [&](ProcessingFunction fn){
        TokenPtr token = output->getAddedToken();
        ASSERT_NE(nullptr, token);

        auto value_msg = std::dynamic_pointer_cast<const connection_types::GenericValueMessage<std::string>>(token->getTokenData());
        ASSERT_NE(nullptr, value_msg);

        ASSERT_EQ("1", value_msg->value);

        importer_node->process(*importer->getNodeHandle(), *importer_node, [&](ProcessingFunction fn){
            TokenPtr token = output->getAddedToken();
            ASSERT_NE(nullptr, token);

            auto value_msg = std::dynamic_pointer_cast<const connection_types::GenericValueMessage<std::string>>(token->getTokenData());
            ASSERT_NE(nullptr, value_msg);

            ASSERT_EQ("2", value_msg->value);

            importer_node->process(*importer->getNodeHandle(), *importer_node, [&](ProcessingFunction fn){
                TokenPtr token = output->getAddedToken();
                ASSERT_NE(nullptr, token);

                auto value_msg = std::dynamic_pointer_cast<const connection_types::GenericValueMessage<std::string>>(token->getTokenData());
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
        NodeFacadeImplementationPtr importer = factory.makeNode("csapex::FileImporter", importer_id, graph);
        ASSERT_NE(nullptr, importer);
        main_graph_facade.addNode(importer);

        importer->setParameter("import directory", true);
        importer->setParameter("directory", std::string(TEST_RES_DIR) + "/input_sequence");

        ASSERT_TRUE(importer->getNode()->hasChangedParameters());
        importer->handleChangedParameters();
        ASSERT_TRUE(importer->canProcess());

        OutputPtr output = importer->getNodeHandle()->getOutput(UUIDProvider::makeDerivedUUID_forced(importer_id, "out_0"));
        ASSERT_NE(nullptr, output);

        // sinks
        NodeFacadeImplementationPtr sink1 = factory.makeNode("AnySink", graph->generateUUID("sink"), graph);
        ASSERT_NE(nullptr, sink1);
        main_graph_facade.addNode(sink1);
        InputPtr input1 = sink1->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(sink1->getUUID(), "in_0"));
        ASSERT_NE(nullptr, input1);
        main_graph_facade.connect(importer, 0, sink1, 0);

        NodeFacadeImplementationPtr sink2 = factory.makeNode("AnySink", graph->generateUUID("sink"), graph);
        ASSERT_NE(nullptr, sink2);
        main_graph_facade.addNode(sink2);
        InputPtr input2 = sink2->getNodeHandle()->getInput(UUIDProvider::makeDerivedUUID_forced(sink2->getUUID(), "in_0"));
        ASSERT_NE(nullptr, input2);
        main_graph_facade.connect(importer, 0, sink2, 0);

        GraphIO io(main_graph_facade, &factory, true);
        ASSERT_NO_THROW(io.saveGraphTo(store));
    }

    {
        auto graph_node = std::make_shared<SubgraphNode>(std::make_shared<GraphImplementation>());
        auto graph = graph_node->getLocalGraph();
        GraphFacadeImplementation main_graph_facade(executor, graph, graph_node);

        GraphIO io(main_graph_facade, &factory, true);
        ASSERT_NO_THROW(io.loadGraphFrom(store));

        NodeFacadeImplementationPtr node = std::dynamic_pointer_cast<NodeFacadeImplementation>(main_graph_facade.findNodeFacade(importer_id));
        ASSERT_NE(nullptr, node);

        OutputPtr out = node->getNodeHandle()->getOutput(UUIDProvider::makeDerivedUUID_forced(importer_id, "out_0"));
        ASSERT_NE(nullptr, out);

        ASSERT_EQ(2, out->getConnections().size());
    }
}

}

