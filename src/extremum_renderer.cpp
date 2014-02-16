/// HEADER
#include "extremum_renderer.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::ExtremumRenderer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace {
const cv::Point3f red(0,0,255);                     /// P0
const cv::Point3f green(0,255,255);                 /// P1
const cv::Point3f blue(255,0,0);                    /// p3
const cv::Point3f fac1 = (blue - 2 * green + red);
const cv::Point3f fac2 = (-2*blue + 2 * green);

inline cv::Scalar maximumColor(const float value)
{
    cv::Point3f  col = fac1 * value * value + fac2 * value + blue;
    return cv::Scalar(std::floor(col.x + .5), std::floor(col.y + .5), std::floor(col.z + .5));
}

}

ExtremumRenderer::ExtremumRenderer()
{
    Tag::createIfNotExists("Visualization");
    addTag(Tag::get("Visualization"));
    addTag(Tag::get("Vision"));
}

void ExtremumRenderer::allConnectorsArrived()
{
    throw std::runtime_error("Not implemented yet!");

    /***
     * To be implemented !
     ***/

    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    out->value = cv::Mat(in->value.rows, in->value.cols, CV_8UC3, cv::Scalar::all(0));
    output_->publish(out);
}

void ExtremumRenderer::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Matrix");
    output_ = addOutput<CvMatMessage>("Extrema");

    update();
}

void ExtremumRenderer::update()
{

}
