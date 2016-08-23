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

class LABToLCh : public csapex::Node
{
public:
    LABToLCh() = default;

    virtual void process() override
    {
        CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
        CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::LCh, in->stamp_micro_seconds));

        const Encoding &encoding = in->getEncoding();

        if(!encoding.matches(enc::lab)) {
            throw std::runtime_error("Need an LAB encoded image!");
        }

        /// L stays as it is
        /// C_ab = sqrt(a^2 + b^2)
        /// H_ab = arctan(b,a) -> exchanged by arctan2
        const cv::Mat &lab = in->value;
        cv::Mat &lch = out->value;
        lch = cv::Mat(lab.rows, lab.cols, CV_32FC3, cv::Scalar());

        float *lch_ptr = lch.ptr<float>();
        const float norm_c = 1.f / (127.0 * sqrt(2.0));
        const float norm_h = 1.f / (2 * M_PI);
        const std::size_t size = lab.rows * lab.cols;

        switch(in->value.type()) {
        case CV_8UC3: {
            uchar const* lab_ptr = lab.ptr<uchar>();
            for(std::size_t i = 0 ; i < size ; ++i) {
                lch_ptr[0]  = lab_ptr[0] / 255.f;
                const float a = (lab_ptr[1] - 128.f); /// opencv specific offset for 8UC1
                const float b = (lab_ptr[2] - 128.f); /// opencv specific offset for 8UC1
                if(b != 0.f || a != 0.f) {
                    /// C
                    lch_ptr[1] = sqrt(a * a + b * b)  * norm_c;
                    /// h
                    lch_ptr[2] = (atan2(b, a) + M_PI) * norm_h;
                }
                lab_ptr += 3;
                lch_ptr += 3;
            }
        }
            break;
        case CV_32FC3: {
            float const *lab_ptr = lab.ptr<float>();
            for(std::size_t i = 0 ; i < size ; ++i) {
                lch_ptr[0]  = lab_ptr[0] / 255.f;
                const float a = (lab_ptr[1] - 128.f); /// opencv specific offset for 8UC1
                const float b = (lab_ptr[2] - 128.f); /// opencv specific offset for 8UC1
                if(b != 0.f || a != 0.f) {
                    /// C
                    lch_ptr[1] = sqrt(a * a + b * b)  * norm_c;
                    /// h
                    lch_ptr[2] = (atan2(b, a) + M_PI) * norm_h;
                }
                lab_ptr += 3;
                lch_ptr += 3;
            }
        }
            break;
        default:
            throw std::runtime_error("Unsupported image depth!");
        }

        msg::publish(output_, out);
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<CvMatMessage>("original");
        output_ = node_modifier.addOutput<CvMatMessage>("adjusted");
    }

    virtual void setupParameters(Parameterizable &parameters)
    {
    }

protected:
    csapex::Output*   output_;
    csapex::Input*    input_;
};
}

CSAPEX_REGISTER_CLASS(csapex::LABToLCh, csapex::Node)
