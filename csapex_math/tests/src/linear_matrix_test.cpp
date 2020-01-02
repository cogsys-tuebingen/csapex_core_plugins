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

#include <csapex_math/msg/linear_matrix_message.h>
#include <csapex_math/param/factory.h>
#include <csapex_math/param/linear_matrix_parameter.h>

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
class MatrixOutputNode
{
public:
    MatrixOutputNode()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        output_ = node_modifier.addOutput<LinearMatrixMessage>("out_vector");
    }

    void setupParameters(Parameterizable& parameters)
    {
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters)
    {
        msg::publish(output_, std::make_shared<LinearMatrixMessage>(latest_vector));
    }

public:
    LinearMatrixMessage latest_vector;

private:
    Output* output_;
};

class MatrixInputNode
{
public:
    MatrixInputNode()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_ = node_modifier.addInput<LinearMatrixMessage>("in_vector");
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::factory::declareMatrix("matrix_parameter", math::linear::Matrix()));
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters)
    {
        latest_vector = msg::getMessage<LinearMatrixMessage>(input_);
        parameters.setParameter("matrix_parameter", latest_vector->value);
    }

public:
    std::shared_ptr<const LinearMatrixMessage> latest_vector;

private:
    Input* input_;
};

class MatrixParamNode
{
public:
    MatrixParamNode()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::factory::declareMatrix("matrix_parameter", math::linear::Matrix()));
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters)
    {
    }
};

class LinearMatrixTest : public SteppingTest
{
public:
    LinearMatrixTest()
    {
        factory.registerNodeType(std::make_shared<NodeConstructor>("MatrixOutputNode", std::bind(&LinearMatrixTest::makeSource)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("MatrixInputNode", std::bind(&LinearMatrixTest::makeSink)));
        factory.registerNodeType(std::make_shared<NodeConstructor>("MatrixParamNode", std::bind(&LinearMatrixTest::makeParamSink)));
    }

private:
    static NodePtr makeSource()
    {
        return NodePtr(new NodeWrapper<MatrixOutputNode>());
    }
    static NodePtr makeSink()
    {
        return NodePtr(new NodeWrapper<MatrixInputNode>());
    }
    static NodePtr makeParamSink()
    {
        return NodePtr(new NodeWrapper<MatrixParamNode>());
    }

private:
    SettingsImplementation settings;
    PluginLocatorPtr locator;

    TestExceptionHandler eh;
    csapex::CsApexCorePtr core;
};

}  // namespace

TEST_F(LinearMatrixTest, MatrixResizeScalesUp)
{
    math::linear::Matrix original(2, 2,
    { 0, 1,
      2, 3 });

    math::linear::Matrix expected(3, 3,
    { 0, 1, 23,
      2, 3, 23,
      23, 23, 23 });

    math::linear::Matrix actual = original;
    actual.resize(3, 3, 23);

    ASSERT_EQ(expected, actual);
}

TEST_F(LinearMatrixTest, MatrixResizeScalesDown)
{
    math::linear::Matrix original(3, 3,
    { 0, 1, 2,
      3, 4, 5,
      6, 7, 8 });

    math::linear::Matrix expected(2, 2,
    { 0, 1,
      3, 4 });

    math::linear::Matrix actual = original;
    actual.resize(2, 2, 23);

    ASSERT_EQ(expected, actual);
}

TEST_F(LinearMatrixTest, MatrixResizeScalesAsymmetric)
{
    math::linear::Matrix original(3, 3,
    { 0, 1, 2,
      3, 4, 5,
      6, 7, 8 });

    math::linear::Matrix expected(1, 5,
    { 0, 1, 2, 23, 23 });

    math::linear::Matrix actual = original;
    actual.resize(1, 5, 23);

    ASSERT_EQ(expected, actual);
}

TEST_F(LinearMatrixTest, MatrixParameterCanBeSerialized)
{
    SerializationBuffer buffer;
    math::linear::Matrix value(2, 2, { 0, 1, 2, 3 });
    {
        param::LinearMatrixParameter p("test", param::ParameterDescription(""), value);
        buffer << p;
    }
    {
        param::LinearMatrixParameter p;
        buffer >> p;

        ASSERT_EQ(p.getValue(), value);
    }
}

TEST_F(LinearMatrixTest, MatrixParameterCanBeSerializedWithYaml)
{
    YAML::Node buffer;
    math::linear::Matrix value(2, 2, { 0, 1, 2, 3 });
    {
        param::LinearMatrixParameter p("test", param::ParameterDescription(""), value);
        p.serialize_yaml(buffer);
    }
    {
        param::LinearMatrixParameter p;
        p.deserialize_yaml(buffer);

        ASSERT_EQ(p.getValue(), value);
    }
}

TEST_F(LinearMatrixTest, MatrixMessageCanCloneDataFromMatrixParameter)
{
    param::LinearMatrixParameter p("test", param::ParameterDescription(""), math::linear::Matrix(2, 2, { 0, 1, 2, 3 }));
    connection_types::LinearMatrixMessage m;
    m.value = math::linear::Matrix(2, 2, { 0, 0, 0, 0 });

    EXPECT_TRUE(p.hasData(typeid(math::linear::Matrix)));
    ASSERT_TRUE(m.hasData(typeid(math::linear::Matrix)));

    ASSERT_NO_THROW(p.cloneDataFrom(m));

    ASSERT_EQ(p.getValue(), m.value);
}

TEST_F(LinearMatrixTest, MatrixParameterCanCloneDataFromMatrixMessage)
{
    param::LinearMatrixParameter p("test", param::ParameterDescription(""), math::linear::Matrix(2, 2, { 0, 1, 2, 3 }));
    connection_types::LinearMatrixMessage m;
    m.value = math::linear::Matrix(2, 2, { 0, 0, 0, 0 });

    EXPECT_TRUE(p.hasData(typeid(math::linear::Matrix)));
    ASSERT_TRUE(m.hasData(typeid(math::linear::Matrix)));

    ASSERT_NO_THROW(m.cloneDataFrom(p));

    ASSERT_EQ(p.getValue(), m.value);
}

TEST_F(LinearMatrixTest, MatrixMessageIsTransmitted)
{
    auto source_p = factory.makeNode("MatrixOutputNode", UUIDProvider::makeUUID_without_parent("MatrixOutputNode"), graph);
    std::shared_ptr<MatrixOutputNode> source = std::dynamic_pointer_cast<MatrixOutputNode>(source_p->getNode());
    ASSERT_NE(nullptr, source);
    main_graph_facade->addNode(source_p);

    auto sink_p = factory.makeNode("MatrixInputNode", UUIDProvider::makeUUID_without_parent("MatrixInputNode"), graph);
    std::shared_ptr<MatrixInputNode> sink = std::dynamic_pointer_cast<MatrixInputNode>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);
    main_graph_facade->addNode(sink_p);

    main_graph_facade->connect(source_p, "out_vector", sink_p, "in_vector");
    ASSERT_TRUE(source_p->canProcess());
    ASSERT_TRUE(sink_p->canProcess());

    executor.start();
    for (int iter = 0; iter < 23; ++iter) {
        math::linear::Matrix v(2, 2, { 0, 1, 2, 3 });
        v(0, 0) = iter;

        source->latest_vector.value = v;

        ASSERT_NO_FATAL_FAILURE(step());

        ASSERT_EQ(v, sink->latest_vector->value);

        auto parameter = sink_p->getParameter("matrix_parameter");
        auto vector_param = std::dynamic_pointer_cast<param::LinearMatrixParameter>(parameter);
        ASSERT_EQ(v, vector_param->getValue());

        ASSERT_EQ(v, parameter->as<math ::linear::Matrix>());
    }
}

TEST_F(LinearMatrixTest, MatrixMessageCanBeConnectedToParameter)
{
    auto source_p = factory.makeNode("MatrixOutputNode", UUIDProvider::makeUUID_without_parent("MatrixOutputNode"), graph);
    std::shared_ptr<MatrixOutputNode> source = std::dynamic_pointer_cast<MatrixOutputNode>(source_p->getNode());
    ASSERT_NE(nullptr, source);
    main_graph_facade->addNode(source_p);

    auto sink_p = factory.makeNode("MatrixParamNode", UUIDProvider::makeUUID_without_parent("MatrixParamNode"), graph);
    std::shared_ptr<MatrixParamNode> sink = std::dynamic_pointer_cast<MatrixParamNode>(sink_p->getNode());
    ASSERT_NE(nullptr, sink);
    main_graph_facade->addNode(sink_p);

    main_graph_facade->connect(source_p, "out_vector", sink_p, "matrix_parameter");
    ASSERT_TRUE(source_p->canProcess());
    ASSERT_TRUE(sink_p->canProcess());

    executor.start();
    for (int iter = 0; iter < 23; ++iter) {
        math::linear::Matrix v(2, 2, { 0, 1, 2, 3 });
        v(0, 0) = iter;

        source->latest_vector.value = v;

        ASSERT_NO_FATAL_FAILURE(step());

        auto parameter = sink_p->getParameter("matrix_parameter");
        auto vector_param = std::dynamic_pointer_cast<param::LinearMatrixParameter>(parameter);
        ASSERT_EQ(v, vector_param->getValue());

        ASSERT_EQ(v, parameter->as<math::linear::Matrix>());
    }
}

}  // namespace csapex
