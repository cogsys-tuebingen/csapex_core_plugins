#include <csapex_testing/stepping_test.h>

#include <csapex/core/core_plugin.h>
#include <csapex/core/csapex_core.h>
#include <csapex/core/graphio.h>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/factory/node_wrapper.hpp>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/io.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/token.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/param/parameter_modifier.h>
#include <csapex/plugin/plugin_locator.h>

#include <csapex_math/msg/linear_vector_message.h>
#include <csapex_math/param/factory.h>
#include <csapex_math/param/linear_vector_parameter.h>

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

namespace
{
class VectorOutputNode
{
public:
    VectorOutputNode()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        output_ = node_modifier.addOutput<LinearVectorMessage>("out_vector");
    }

    void setupParameters(Parameterizable& parameters)
    {
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters)
    {
        msg::publish(output_, std::make_shared<LinearVectorMessage>(latest_vector));
    }

public:
    LinearVectorMessage latest_vector;

private:
    Output* output_;
};

class VectorInputNode
{
public:
    VectorInputNode()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_ = node_modifier.addInput<LinearVectorMessage>("in_vector");
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::factory::declareVector("vector_parameter", std::vector<double>{ 0, 0, 0 }));
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters)
    {
        latest_vector = msg::getMessage<LinearVectorMessage>(input_);
        parameters.setParameter("vector_parameter", latest_vector->value);
    }

public:
    std::shared_ptr<const LinearVectorMessage> latest_vector;

private:
    Input* input_;
};

class VectorParamNode
{
public:
    VectorParamNode()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::factory::declareVector("vector_parameter", std::vector<double>{ 0, 0, 0 }));
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters)
    {
    }
};

class LinearVectorTest : public SteppingTest
{
public:
    LinearVectorTest()
    {
        factory.registerNodeType(std::make_shared<NodeConstructor>("VectorOutputNode", std::bind(&LinearVectorTest::makeSource)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("VectorInputNode", std::bind(&LinearVectorTest::makeSink)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("VectorParamNode", std::bind(&LinearVectorTest::makeParamSink)));
    }

private:
    static NodePtr makeSource()
    {
        return NodePtr(new NodeWrapper<VectorOutputNode>());
    }
    static NodePtr makeSink()
    {
        return NodePtr(new NodeWrapper<VectorInputNode>());
    }
    static NodePtr makeParamSink()
    {
        return NodePtr(new NodeWrapper<VectorParamNode>());
    }

private:
    SettingsImplementation settings;
    PluginLocatorPtr locator;

    TestExceptionHandler eh;
    csapex::CsApexCorePtr core;
};

}  // namespace

TEST_F(LinearVectorTest, VectorParameterCanBeSerialized)
{
    SerializationBuffer buffer;
    math::linear::Vector value({ 1, 2, 3 });
    {
        param::LinearVectorParameter p("test", param::ParameterDescription(""), value);
        buffer << p;
    }
    {
        param::LinearVectorParameter p;
        buffer >> p;

        ASSERT_EQ(p.getValue(), value);
    }
}

TEST_F(LinearVectorTest, VectorParameterCanBeSerializedWithYaml)
{
    YAML::Node buffer;
    math::linear::Vector value({ 1, 2, 3 });
    {
        param::LinearVectorParameter p("test", param::ParameterDescription(""), value);
        p.serialize_yaml(buffer);
    }
    {
        param::LinearVectorParameter p;
        p.deserialize_yaml(buffer);

        ASSERT_EQ(p.getValue(), value);
    }
}

TEST_F(LinearVectorTest, VectorMessageCanCloneDataFromVectorParameter)
{
    param::LinearVectorParameter p("test", param::ParameterDescription(""), { 1, 2, 3 });
    connection_types::LinearVectorMessage m;
    m.value = math::linear::Vector({ 0, 0, 0 });

    EXPECT_TRUE(p.hasData(typeid(math::linear::Vector)));
    ASSERT_TRUE(m.hasData(typeid(math::linear::Vector)));

    ASSERT_NO_THROW(p.cloneDataFrom(m));

    ASSERT_EQ(p.getValue(), m.value);
}

TEST_F(LinearVectorTest, VectorParameterCanCloneDataFromVectorMessage)
{
    param::LinearVectorParameter p("test", param::ParameterDescription(""), { 1, 2, 3 });
    connection_types::LinearVectorMessage m;
    m.value = math::linear::Vector({ 0, 0, 0 });

    EXPECT_TRUE(p.hasData(typeid(math::linear::Vector)));
    ASSERT_TRUE(m.hasData(typeid(math::linear::Vector)));

    ASSERT_NO_THROW(m.cloneDataFrom(p));

    ASSERT_EQ(p.getValue(), m.value);
}

TEST_F(LinearVectorTest, VectorMessageIsTransmitted)
{
    auto source_p = factory.makeNode("VectorOutputNode", UUIDProvider::makeUUID_without_parent("VectorOutputNode"), graph);
    std::shared_ptr<VectorOutputNode> source = std::dynamic_pointer_cast<VectorOutputNode>(source_p->getNode());
    ASSERT_NE(nullptr, source);
    main_graph_facade->addNode(source_p);

    auto sink_p = factory.makeNode("VectorInputNode", UUIDProvider::makeUUID_without_parent("VectorInputNode"), graph);
    std::shared_ptr<VectorInputNode> sink = std::dynamic_pointer_cast<VectorInputNode>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);
    main_graph_facade->addNode(sink_p);

    main_graph_facade->connect(source_p, "out_vector", sink_p, "in_vector");
    ASSERT_TRUE(source_p->canProcess());
    ASSERT_TRUE(sink_p->canProcess());

    executor.start();
    for (int iter = 0; iter < 23; ++iter) {
        math::linear::Vector v(std::vector<double>{ 1, 2, 3 });
        v(0) = iter;

        source->latest_vector.value = v;

        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(v, sink->latest_vector->value);

        auto parameter = sink_p->getParameter("vector_parameter");
        auto vector_param = std::dynamic_pointer_cast<param::LinearVectorParameter>(parameter);
        ASSERT_EQ(v, vector_param->getValue());

        ASSERT_EQ(v, parameter->as<math ::linear::Vector>());
    }
}

TEST_F(LinearVectorTest, VectorMessageCanBeConnectedToParameter)
{
    auto source_p = factory.makeNode("VectorOutputNode", UUIDProvider::makeUUID_without_parent("VectorOutputNode"), graph);
    std::shared_ptr<VectorOutputNode> source = std::dynamic_pointer_cast<VectorOutputNode>(source_p->getNode());
    ASSERT_NE(nullptr, source);
    main_graph_facade->addNode(source_p);

    auto sink_p = factory.makeNode("VectorParamNode", UUIDProvider::makeUUID_without_parent("VectorParamNode"), graph);
    std::shared_ptr<VectorParamNode> sink = std::dynamic_pointer_cast<VectorParamNode>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);
    main_graph_facade->addNode(sink_p);

    main_graph_facade->connect(source_p, "out_vector", sink_p, "vector_parameter");
    ASSERT_TRUE(source_p->canProcess());
    ASSERT_TRUE(sink_p->canProcess());

    executor.start();
    for (int iter = 0; iter < 23; ++iter) {
        math::linear::Vector v(std::vector<double>{ 1, 2, 3 });
        v(0) = iter;

        source->latest_vector.value = v;

        ASSERT_NO_FATAL_FAILURE(step());

        auto parameter = sink_p->getParameter("vector_parameter");
        auto vector_param = std::dynamic_pointer_cast<param::LinearVectorParameter>(parameter);
        ASSERT_EQ(v, vector_param->getValue());

        ASSERT_EQ(v, parameter->as<math::linear::Vector>());
    }
}

}  // namespace csapex
