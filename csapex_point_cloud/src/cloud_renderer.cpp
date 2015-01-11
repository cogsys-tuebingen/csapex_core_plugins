/// HEADER
#include "cloud_renderer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::CloudRenderer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

CloudRenderer::CloudRenderer()
{

}

void CloudRenderer::setupParameters()
{
    boost::function<void(param::Parameter*)> refresh = boost::bind(&CloudRenderer::refresh, this);

    double d = 10.0;

    addParameter(param::ParameterFactory::declareRange("~view/r", 0.01, 20.0, 10.0, 0.01), refresh);
    addParameter(param::ParameterFactory::declareRange("~view/theta", 0., M_PI, M_PI / 2, 0.001), refresh);
    addParameter(param::ParameterFactory::declareRange("~view/phi", -M_PI, M_PI, 0., 0.001), refresh);
    addParameter(param::ParameterFactory::declareRange("~view/dx", -d, d, 0., 0.01), refresh);
    addParameter(param::ParameterFactory::declareRange("~view/dy", -d, d, 0., 0.01), refresh);
    addParameter(param::ParameterFactory::declareRange("~view/dz", -d, d, 0., 0.01), refresh);

    addParameter(param::ParameterFactory::declareRange("~size/width", 10, 1024, 400, 1), refresh);
    addParameter(param::ParameterFactory::declareRange("~size/height", 10, 1024, 400, 1), refresh);

    param::Parameter::Ptr sync = param::ParameterFactory::declareBool("~size/out/sync", true);
    addParameter(sync, refresh);
    boost::function<bool()> notsync = (!boost::bind(&param::Parameter::as<bool>, sync.get()));
    addConditionalParameter(param::ParameterFactory::declareRange("~size/out/width", 10, 1024, 400, 1), notsync, refresh);
    addConditionalParameter(param::ParameterFactory::declareRange("~size/out/height", 10, 1024, 400, 1), notsync, refresh);

    addParameter(param::ParameterFactory::declareColorParameter("color/background", 255, 255, 255), refresh);
    addParameter(param::ParameterFactory::declareColorParameter("color/grid", 0, 0, 0), refresh);
    addParameter(param::ParameterFactory::declareColorParameter("color/gradient/start", 0, 255, 0), refresh);
    addParameter(param::ParameterFactory::declareColorParameter("color/gradient/end", 0, 0, 255), refresh);
    addParameter(param::ParameterFactory::declareBool("color/rainbow",
                                                      param::ParameterDescription("Sample from a gradient of rainbow colors"),
                                                      false),
                 refresh);
    addParameter(param::ParameterFactory::declareBool("color/force gradient", false), refresh);

    std::vector<std::string> field;
    field.push_back("x");
    field.push_back("y");
    field.push_back("z");
    field.push_back("i");
    addParameter(param::ParameterFactory::declareParameterStringSet("color/field", field, "x"), refresh);

    addParameter(param::ParameterFactory::declareBool("show axes", false), refresh);

    addParameter(param::ParameterFactory::declareRange("~grid/size", 1, 30, 10, 1), refresh);
    addParameter(param::ParameterFactory::declareRange("~grid/resolution", 0.1, 10.0, 1.0, 0.1), refresh);
    addParameter(param::ParameterFactory::declareBool("~grid/xy", true), refresh);
    addParameter(param::ParameterFactory::declareBool("~grid/yz", false), refresh);
    addParameter(param::ParameterFactory::declareBool("~grid/xz", false), refresh);

    addParameter(param::ParameterFactory::declareRange("point/size", 1., 30., 5., 0.1), refresh);
}

void CloudRenderer::setup()
{
    input_ = modifier_->addInput<PointCloudMessage>("PointCloud");
    output_ = modifier_->addOutput<CvMatMessage>("Rendered Image");
}

void CloudRenderer::process()
{
    InteractiveNode::process();

    if(!result_) {
        refresh_request();
    }

    PointCloudMessage::ConstPtr msg = input_->getMessage<PointCloudMessage>();

    message_ = msg;
    result_.reset();

    display_request();

    // todo wait only in !headless

    if(output_->isConnected()) {
        if(waitForView()) {
            if(result_) {
                output_->publish(result_);
            }
        }
    }
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
