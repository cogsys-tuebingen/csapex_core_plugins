/// HEADER
#include "cloud_renderer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::CloudRenderer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

CloudRenderer::CloudRenderer()
{

}

void CloudRenderer::setupParameters(Parameterizable& parameters)
{
    std::function<void(csapex::param::Parameter*)> refresh = std::bind(&CloudRenderer::refresh, this);

    double d = 10.0;

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/r", 0.01, 20.0, 10.0, 0.01), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/theta", 0., M_PI, M_PI / 2, 0.001), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/phi", -M_PI, M_PI, 0., 0.001), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/dx", -d, d, 0., 0.01), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/dy", -d, d, 0., 0.01), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/dz", -d, d, 0., 0.01), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~size/width", 10, 1024, 400, 1), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~size/height", 10, 1024, 400, 1), refresh);

    csapex::param::Parameter::Ptr sync = csapex::param::ParameterFactory::declareBool("~size/out/sync", true);
    parameters.addParameter(sync, refresh);
    std::function<bool()> notsync = [sync]() { return !sync->as<bool>(); };
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange("~size/out/width", 10, 1024, 400, 1), notsync, refresh);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange("~size/out/height", 10, 1024, 400, 1), notsync, refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color/background", 255, 255, 255), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color/grid", 0, 0, 0), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color/gradient/start", 0, 255, 0), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color/gradient/end", 0, 0, 255), refresh);
    addParameter(csapex::param::ParameterFactory::declareBool("color/rainbow",
                                                      csapex::param::ParameterDescription("Sample from a gradient of rainbow colors"),
                                                      false),
                 refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("color/force gradient", false), refresh);

    std::vector<std::string> field;
    field.push_back("x");
    field.push_back("y");
    field.push_back("z");
    field.push_back("i");
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterStringSet("color/field", field, "x"), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareBool("show axes", false), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~grid/size", 1, 30, 10, 1), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~grid/resolution", 0.1, 10.0, 1.0, 0.1), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("~grid/xy", true), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("~grid/yz", false), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("~grid/xz", false), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("point/size", 1., 30., 5., 0.1), refresh);
}

void CloudRenderer::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    output_ = node_modifier.addOutput<CvMatMessage>("Rendered Image");
}

void CloudRenderer::beginProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    if(!result_) {
        refresh();
    }

    PointCloudMessage::ConstPtr msg = msg::getMessage<PointCloudMessage>(input_);

    {
        std::unique_lock<std::mutex> lock(message_mutex_);
        message_ = msg;

        result_.reset();
    }

    display_request();

    if(!msg::isConnected(output_)) {
        done();
    }
}

void CloudRenderer::finishProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    if(msg::isConnected(output_)) {
        if(result_) {
            msg::publish(output_, result_);
        }
    }
}

connection_types::PointCloudMessage::ConstPtr CloudRenderer::getMessage() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return message_;
}

bool CloudRenderer::isOutputConnected() const
{
    return msg::isConnected(output_);
}

void CloudRenderer::refresh()
{
    refresh_request();
}

void CloudRenderer::publishImage(const cv::Mat &img)
{
    CvMatMessage::Ptr msg(new CvMatMessage(enc::bgr, 0));
    img.copyTo(msg->value);

    result_ = msg;

    done();
}
