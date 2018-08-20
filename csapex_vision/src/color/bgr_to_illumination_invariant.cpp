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
        case NewmanOriginal:
        {
          const cv::Vec3b *src = in->value.ptr<cv::Vec3b>();
          float *dst  = out->value.ptr<float>();
          uchar *mask = out_mask->value.ptr<uchar>();
          const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

          for(std::size_t i = 0 ; i < size ; ++i) {
              const cv::Vec3b &p = src[i];
              if(p[0] == 0 || p[1] == 0 || p[2] == 0) {
                  mask[i] = 0;
                  continue;
              }
              cv::Vec3f c(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0);
              dst[i] = 0.5 + std::log(c[1]) - alpha_ * std::log(c[0]) - (1.0 - alpha_) * std::log(c[2]);
          }
        }
            break;
        case Newman:
        {
            double min = 0.5 - std::log(255.0);
            double max = 0.5 + std::log(255.0);

            const cv::Vec3b *src = in->value.ptr<cv::Vec3b>();
            float *dst  = out->value.ptr<float>();
            uchar *mask = out_mask->value.ptr<uchar>();
            const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

            for(std::size_t i = 0 ; i < size ; ++i) {
                const cv::Vec3b &p = src[i];
                cv::Vec3f c(p);
                if(p[0] == 0 || p[1] == 0 || p[2] == 0) {
                    mask[i] = 0;
                    continue;
                }
                dst[i] = (0.5 + std::log(c[1]) - alpha_ * std::log(c[0]) - (1.0 - alpha_) * std::log(c[2]));
                dst[i] = ((dst[i] - min) / (max - min)); //normalize 0-1
//                dst[i] = dst[i] * 255.0; //discretize 0-255
            }
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

            double max = std::log(255.0) * (0.5 + (1 / std::sqrt(3)));
            double min = - max;

            for(std::size_t i = 0 ; i < size ; ++i) {
                const cv::Vec3b &p = src[i];
                if(p[0] == 0 || p[1] == 0 || p[2] == 0) {
                    mask[i] = 0;
                    dst[i] = 0;
                    continue;
                }

                cv::Vec3f c(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0);

                const double rho = std::pow((c[0] * c[1] * c[2]), 1.0 / 3);
                c /= rho;
                c[0] = std::log(c[0]);
                c[1] = std::log(c[1]);
                c[2] = std::log(c[2]);
                const cv::Vec2f psi(c.ddot(v1), c.ddot(v2));
                dst[i] = ((psi[0] * ct + psi[1]  * st) - min) / (max - min); //normalize
//                dst[i] = dst[i] * 255.0; //discretize 0-255
            }
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
            {"NewmanOriginal", (int) NewmanOriginal},
            {"Santos", (int) Santos},
            {"SantosOriginal", (int) SantosOriginal}
        };

        parameters.addParameter(param::ParameterFactory::declareParameterSet<int>("method", methods, Newman),
                                method_);

    }

protected:
    csapex::Output*   output_;
    csapex::Output*   output_mask_;

    enum Type {Newman, NewmanOriginal, Santos, SantosOriginal};

    csapex::Input*    input_;
    double            alpha_;
    int               method_;
    bool              recover_bgr_;
};
}

CSAPEX_REGISTER_CLASS(csapex::BGRToIlluminationInvariant, csapex::Node)
