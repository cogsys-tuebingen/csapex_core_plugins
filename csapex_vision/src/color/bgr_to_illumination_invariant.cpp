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

        switch(method_) {
        case Newman:
        {
            const cv::Vec3b *src = in->value.ptr<cv::Vec3b>();
            float *dst  = out->value.ptr<float>();
            uchar *mask = out_mask->value.ptr<uchar>();
            const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);
            double min = 2 * std::log(1.0 / 255.0);

            for(std::size_t i = 0 ; i < size ; ++i) {
                const cv::Vec3b &p = src[i];
                if(p[0] == 0 || p[1] == 0 || p[2] == 0) {
                    mask[i] = 0;

                }
                cv::Vec3f c(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0);
                dst[i] = (0.5 + std::log(c[1]) - alpha_ * std::log(c[0]) - (1.0 - alpha_) * std::log(c[2]) + min) / (-min + 0.5);
            }

            /// min = 2 * std::log(1. / 255.)
            /// max = 2 * std::log(1.0)
        }
            break;
        case Santos:
        {
            const cv::Vec3b *src = in->value.ptr<cv::Vec3b>();
            float *dst  = out->value.ptr<float>();
            uchar *mask = out_mask->value.ptr<uchar>();
            const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

            const double theta = M_PI * alpha_;
            const double ct = std::cos(theta);
            const double st = std::sin(theta);

            const static cv::Vec3f v1(1.0 / std::sqrt(2.0), -1.0 / std::sqrt(2.0), 0.0);
            const static cv::Vec3f v2(1.0 / std::sqrt(6.0),  1.0 / std::sqrt(6.0), -2.0 / std::sqrt(6.0));  //// Entropy Minimization for Shadow Removal

            // double min = 0.0;
            double max = std::log(1.0 / std::sqrt(1.0 / 255.0)) / std::sqrt(2.0);

            for(std::size_t i = 0 ; i < size ; ++i) {
                const cv::Vec3b &p = src[i];
                if(p[0] == 0 && p[1] == 0 && p[2] == 0) {
                    mask[i] = 0;
                    dst[i] = 0;
                    continue;
                }
                cv::Vec3f c(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0);
                const double rho = std::sqrt(c.ddot(c));
                c /= rho;
                const cv::Vec2f psi(c.ddot(v1), c.ddot(v2));
                dst[i] = (psi[0] * ct + psi[1] * st) / max;
            }

            /// sqrt2(1) <=> sqrt2(1/255.)
            /// ct + st <= sqrt(2)
        }
            break;

        case SantosOriginal:
        {
            const cv::Vec3b *src = in->value.ptr<cv::Vec3b>();
            float *dst  = out->value.ptr<float>();
            uchar *mask = out_mask->value.ptr<uchar>();
            const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

            const double theta = M_PI * alpha_;
            const double ct = std::cos(theta);
            const double st = std::sin(theta);
            const static cv::Vec3f v1(1.0 / std::sqrt(2.0), -1.0 / std::sqrt(2.0), 0.0);
            const static cv::Vec3f v2(1.0 / std::sqrt(6.0),  1.0 / std::sqrt(6.0), -2.0 / std::sqrt(6.0));  //// Entropy Minimization for Shadow Removal

            double max = std::log(1.0 / std::pow(((1.0 / 255.0) * (1.0 / 255.0) * (1.0 / 255.0)), 1.0 / 3.0)) / std::sqrt(2.0);
            for(std::size_t i = 0 ; i < size ; ++i) {
                const cv::Vec3b &p = src[i];
                if(p[0] == 0 || p[1] == 0 || p[2] == 0) {
                    mask[i] = 0;
                    dst[i] = 0;
                    continue;
                }
                cv::Vec3f c(p[2] / 255.0, p[1] / 255.0, p[1] / 255.0);
                const double rho = std::pow((c[0] * c[1] * c[2]), 1.0 / 3);
                c /= rho;
                const cv::Vec2f psi(c.ddot(v1), c.ddot(v2));
                dst[i] = (psi[0] * ct + psi[1]  * st) / max;
            }
            /// sqrt3(1) <=> sqrt3((1/255)^3) < 1.0
            /// ct + st <= sqrt(2)
        }
            break;
        default:
            break;
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
        std::map<std::string, int> methods = {
            {"Newman", (int) Newman},
            {"Santos", (int) Santos},
            {"SantosOriginal", (int) SantosOriginal}
        };

        parameters.addParameter(param::ParameterFactory::declareParameterSet<int>("method", methods, Newman),
                                method_);

    }

protected:
    csapex::Output*   output_;
    csapex::Output*   output_mask_;

    enum Type {Newman, Santos, SantosOriginal};

    csapex::Input*    input_;
    double            alpha_;
    int               method_;
    bool              recover_bgr_;
};
}

CSAPEX_REGISTER_CLASS(csapex::BGRToIlluminationInvariant, csapex::Node)
