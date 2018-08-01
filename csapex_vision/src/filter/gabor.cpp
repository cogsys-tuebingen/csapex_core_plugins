#include "gabor.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <opencv2/imgproc/imgproc.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::Gabor, csapex::Node)

Gabor::Gabor()
{
}

void Gabor::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

    cv::Mat kernel = cv::getGaborKernel(cv::Size(ksize_,ksize_),sigma_, theta_, lambda_, gamma_, psi_);
    cv::filter2D(in->value, out->value, in->value.depth(), kernel);
    msg::publish(output_, out);
}

void Gabor::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("Gabord");
}

void Gabor::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareRange("ksize",
                                                                   param::ParameterDescription("Kernel size."),
                                                                   1, 23, 3, 2),
                            ksize_);

    parameters.addParameter(param::factory::declareRange("sigma",
                                                                   param::ParameterDescription("Standard deviation of the gaussian envelope."),
                                                                   0.01, 10.0, 0.5, 0.01),
                            sigma_);

    parameters.addParameter(param::factory::declareAngle("theta",
                                                                   param::ParameterDescription("Orientation of the normal to the parallel stripes of a Gabor function."),
                                                                   0.0),
                            theta_);

    parameters.addParameter(param::factory::declareRange("lambda",
                                                                   param::ParameterDescription("Wavelength of the sinusoidal factor."),
                                                                   1.0, 100.0, 1.0, 0.01),
                            lambda_);

    parameters.addParameter(param::factory::declareRange("gamma",
                                                                   param::ParameterDescription("Spatial aspect ratio."),
                                                                   1.0, 100.0, 1.0, 0.01),
                            gamma_);

    parameters.addParameter(param::factory::declareRange("psi",
                                                                   param::ParameterDescription("Phase offset."),
                                                                   1.0, 100.0, 1.0, 0.01),
                            psi_);

}
