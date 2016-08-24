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
    LABToLCh()
    {
        LUT_sqrt = cv::Mat(255, 255, CV_32FC1, cv::Scalar());
        LUT_atan2 = cv::Mat(255, 255, CV_32FC1, cv::Scalar());

        float *LUT_sqrt_ptr = LUT_sqrt.ptr<float>();
        float *LUT_atan2_ptr = LUT_atan2.ptr<float>();
        for(int i = 0 ; i < 255 ; ++i) {
            for(int j = 0 ; j < 255 ; ++j) {
                const float a = i - 128.f;
                const float b = j - 128.f;
                if(a == 0 && b == 0)
                    continue;
                LUT_sqrt_ptr[i * 255 + j]  = sqrt(a * a + b * b) * NORM_C;
                LUT_atan2_ptr[i * 255 + j] = (atan2(b, a) + M_PI) * NORM_H;
            }
        }

    }

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

        switch(in->value.type()) {
        case CV_8UC3:
            processCV_8UC3(in->value, out->value);
            break;
        case CV_32FC3:
            processCV_32FC3(in->value, out->value);
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

    static constexpr float NORM_L = 1.f / 255.f;
    static constexpr float NORM_C = 1.f / (127.0 * sqrt(2.0));
    static constexpr float NORM_H = 1.f / (2 * M_PI);
    cv::Mat LUT_sqrt;
    cv::Mat LUT_atan2;

    inline void processCV_8UC3(const cv::Mat &src, cv::Mat &dst)
    {
        dst = cv::Mat(src.rows, src.cols, CV_32FC3, cv::Scalar());

        float *dst_ptr = dst.ptr<float>();
        uchar const *src_ptr = src.ptr<uchar>();
        const float *LUT_sqrt_ptr = LUT_sqrt.ptr<float>();
        const float *LUT_atan2_ptr = LUT_atan2.ptr<float>();

        const std::size_t size = src.rows * src.cols;
        for(std::size_t i = 0 ; i < size ; ++i) {
            dst_ptr[0] = src_ptr[0] * NORM_L;
            const int a = src_ptr[1];
            const int b = src_ptr[2];
            dst_ptr[1] = LUT_sqrt_ptr[255 * a + b];
            dst_ptr[2] = LUT_atan2_ptr[255 * a + b];
            dst_ptr += 3;
            src_ptr += 3;
        }
    }

    inline void processCV_32FC3(const cv::Mat &src, cv::Mat &dst)
    {
        dst = cv::Mat(src.rows, src.cols, CV_32FC3, cv::Scalar());

        float *dst_ptr = dst.ptr<float>();
        float const *src_ptr = src.ptr<float>();
        const float *LUT_sqrt_ptr = LUT_sqrt.ptr<float>();
        const float *LUT_atan2_ptr = LUT_atan2.ptr<float>();

        const std::size_t size = src.rows * src.cols;
        for(std::size_t i = 0 ; i < size ; ++i) {
            dst_ptr[0] = src_ptr[0] * NORM_L;
            const int a = src_ptr[1] + 128.f;
            const int b = src_ptr[2] + 128.f;
            dst_ptr[1] = LUT_sqrt_ptr[255 * a + b];
            dst_ptr[2] = LUT_atan2_ptr[255 * a + b];
            dst_ptr += 3;
            src_ptr += 3;
        }
    }


};
}

CSAPEX_REGISTER_CLASS(csapex::LABToLCh, csapex::Node)
