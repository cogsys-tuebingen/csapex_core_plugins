/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

namespace csapex
{
using namespace csapex;
using namespace connection_types;

class NewmanIlluminationInvariance : public csapex::Node
{
public:
    NewmanIlluminationInvariance() = default;
    virtual ~NewmanIlluminationInvariance() = default;

    virtual void process() override
    {
        CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
        if (!in->getEncoding().matches(enc::bgr))
            throw std::runtime_error("Image needs to be BGR for fitting conversion.");



        CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds));
        const cv::Mat bgr = in->value;

        cv::Mat bgr_f;
        bgr.convertTo(bgr_f, CV_32F, 1.0, 1.0);
        cv::Mat lbgr_f;
        cv::log(bgr_f,lbgr_f);

        std::vector<cv::Mat> bgr_channels;
        cv::split(lbgr_f, bgr_channels);

        cv::Mat &b = bgr_channels[0];
        cv::Mat &g = bgr_channels[1];
        cv::Mat &r = bgr_channels[2];

        out->value = 0.5 + g - alpha_ * b - (1.0 - alpha_) * r;
        out->value /= std::log(256.0);

        msg::publish(output_, out);
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<CvMatMessage>("original");
        output_ = node_modifier.addOutput<CvMatMessage>("adjusted");
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("alpha", 0.0, 1.0, 1.5, 0.001), alpha_);
    }

protected:
    csapex::Output* output_;
    csapex::Input* input_;
    double alpha_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::NewmanIlluminationInvariance, csapex::Node)
