/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

namespace csapex {
using namespace csapex;
using namespace connection_types;

class BGRToIlluminationInvariant : public csapex::Node
{
public:
    BGRToIlluminationInvariant() = default;
    virtual ~BGRToIlluminationInvariant() = default;

    virtual void process() override
    {
        CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
        if(!in->getEncoding().matches(enc::bgr))
          throw std::runtime_error("Image needs to be BGR for fitting conversion.");

        CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::unknown, in->frame_id, in->stamp_micro_seconds));
        out->value = cv::Mat(in->value.rows, in->value.cols, CV_32FC1, cv::Scalar());
        CvMatMessage::Ptr out_mask(new connection_types::CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds));
        out_mask->value = cv::Mat(in->value.rows, in->value.cols, CV_8UC1, cv::Scalar(255));


        const cv::Vec3b *src = in->value.ptr<cv::Vec3b>();
        float *dst  = out->value.ptr<float>();
        uchar *mask = out_mask->value.ptr<uchar>();
        const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);
        for(std::size_t i = 0 ; i < size ; ++i) {
            const cv::Vec3b &p = src[i];
            if(p[0] == 0 || p[1] == 0 || p[2] == 0) {
              mask[i] = 0;
            }
            dst[i] = 0.5 + std::log(p[1]) - alpha_ * std::log(p[0]) - (1.0 - alpha_) * std::log(p[2]);
        }

        msg::publish(output_, out);
        msg::publish(output_mask_, out_mask);
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<CvMatMessage>("original");
        output_ = node_modifier.addOutput<CvMatMessage>("adjusted");
        output_mask_ = node_modifier.addOutput<CvMatMessage>("mask");
    }

    virtual void setupParameters(Parameterizable &parameters)
    {
        parameters.addParameter(param::ParameterFactory::declareRange("alpha", 0.0, 1.0, 1.5, 0.001),
                                alpha_);
        parameters.addParameter(param::ParameterFactory::declareBool("normalize", true),
                                normalize_);
    }

protected:
    csapex::Output*   output_;
    csapex::Output*   output_mask_;

    csapex::Input*    input_;
    double alpha_;
    bool normalize_;
};
}

CSAPEX_REGISTER_CLASS(csapex::BGRToIlluminationInvariant, csapex::Node)
