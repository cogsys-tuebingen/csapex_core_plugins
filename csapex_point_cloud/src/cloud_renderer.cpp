/// HEADER
#include "cloud_renderer.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::CloudRenderer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

CloudRenderer::CloudRenderer()
    : stopped_(false)
{
    addTag(Tag::get("PointCloud"));

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

    addParameter(param::ParameterFactory::declareColorParameter("color/background", 0, 0, 0), refresh);
    addParameter(param::ParameterFactory::declareColorParameter("color/gradient/start", 0, 255, 0), refresh);
    addParameter(param::ParameterFactory::declareColorParameter("color/gradient/end", 0, 0, 255), refresh);
    addParameter(param::ParameterFactory::declareBool("color/force gradient", false), refresh);

    std::vector<std::string> field;
    field.push_back("x");
    field.push_back("y");
    field.push_back("z");
    addParameter(param::ParameterFactory::declareParameterStringSet("color/field", field), refresh);

    addParameter(param::ParameterFactory::declareRange("point/size", 1., 30., 5., 0.1), refresh);
}

void CloudRenderer::setup()
{
    input_ = addInput<PointCloudMessage>("PointCloud");
    output_ = addOutput<CvMatMessage>("Rendered Image");
}

void CloudRenderer::stop()
{
    stopped_ = true;
    wait_for_view_.wakeAll();

    Node::stop();
}

void CloudRenderer::process()
{
    if(!result_) {
        refresh_request();
    }

    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    message_ = msg;
    result_.reset();

    display_request();

    // todo wait only in !headless

    if(output_->isConnected()) {
        result_mutex_.lock();
        while(!result_ && !stopped_) {
            wait_for_view_.wait(&result_mutex_);
        }
        result_mutex_.unlock();

        if(result_) {
            output_->publish(result_);
        }
    }
}


void CloudRenderer::refresh()
{
    refresh_request();
}

void CloudRenderer::publishImage(const cv::Mat &img)
{
    {
        QMutexLocker lock(&result_mutex_);

        CvMatMessage::Ptr msg(new CvMatMessage(enc::bgr));
        img.copyTo(msg->value);

        result_ = msg;
        wait_for_view_.wakeAll();
    }

}
