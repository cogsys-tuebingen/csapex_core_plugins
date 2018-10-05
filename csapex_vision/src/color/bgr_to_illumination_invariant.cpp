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

class BGRToIlluminationInvariant : public csapex::Node
{
public:
    BGRToIlluminationInvariant() = default;
    virtual ~BGRToIlluminationInvariant() = default;

    virtual void process() override
    {
        CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
        if (!in->getEncoding().matches(enc::bgr))
            throw std::runtime_error("Image needs to be BGR for fitting conversion.");

        CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds));
        out->value = cv::Mat(in->value.rows, in->value.cols, CV_32FC1, cv::Scalar());
        CvMatMessage::Ptr out_mask(new connection_types::CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds));
        out_mask->value = cv::Mat(in->value.rows, in->value.cols, CV_8UC1, cv::Scalar(0));

        // bool cap
        // conditional for cap (min/max) interval param
        // mask also with capped pixels
        // ternary after norm for cap (after ill.invar)
        // labels : 0 unaltered, 1 invalid, 2 capped (or 2 for min capped 3 for max
        // capped)
        //--> labels to mask <-> generate mask from label (as parameter)
        //--> labels to random color <-> check examples for random colors

        std::pair<float, float> threshold_min_max = readParameter<std::pair<int, int>>("threshold range");
        threshold_min_max.first = threshold_min_max.first / 100.0;
        threshold_min_max.second = threshold_min_max.second / 100.0;

        bool use_threshold = readParameter<bool>("threshold");

        switch (method_) {
            case NewmanOriginal: {
                const cv::Vec3b* src = in->value.ptr<cv::Vec3b>();
                float* dst = out->value.ptr<float>();
                uchar* mask = out_mask->value.ptr<uchar>();
                const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

                for (std::size_t i = 0; i < size; ++i) {
                    const cv::Vec3b& p = src[i];
                    if (p[0] == 0 || p[1] == 0 || p[2] == 0) {
                        mask[i] = 0;
                        continue;
                    }
                    cv::Vec3f c(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0);
                    dst[i] = 0.5 + std::log(c[1]) - alpha_ * std::log(c[0]) - (1.0 - alpha_) * std::log(c[2]);
                }
            } break;
            case Newman: {
                // N = (0.5+log(g)) - alpha*log(b) - (1-alpha)*log(r)
                // Max= 0.5 + log(255) - alpha*log(b) - log(r) + alpha*log(r) = 0.5 +
                // log(255)   (alpha,b,r = 0) Min= 0.5 + log(1) - alpha*log(b) - log(r) +
                // alpha*log(r) = 0.5 - log(255) (alpha=1,b=255, r=0)
                double min = 0.5 - std::log(255.0);
                double max = 0.5 + std::log(255.0);

                const cv::Vec3b* src = in->value.ptr<cv::Vec3b>();
                float* dst = out->value.ptr<float>();
                uchar* mask = out_mask->value.ptr<uchar>();
                const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

                auto get_thresholded = [&](int i) -> std::pair<double, double> {
                    if (use_threshold) {
                        if (dst[i] < threshold_min_max.first) {
                            return std::make_pair(threshold_min_max.first, 2);
                        } else if (dst[i] > threshold_min_max.second) {
                            return std::make_pair(threshold_min_max.second, 3);
                        }
                    }
                    return std::make_pair(dst[i], mask[i]);
                };

                for (std::size_t i = 0; i < size; ++i) {
                    const cv::Vec3b& p = src[i];
                    cv::Vec3f c(p);
                    if (p[0] == 0 || p[1] == 0 || p[2] == 0) {
                        mask[i] = 0;
                        continue;
                    }
                    dst[i] = (0.5 + std::log(c[1]) - alpha_ * std::log(c[0]) - (1.0 - alpha_) * std::log(c[2]));
                    dst[i] = ((dst[i] - min) / (max - min));         // normalize 0-1
                    std::tie(dst[i], mask[i]) = get_thresholded(i);  // threshold min/max
                                                                     // vals
                    dst[i] = dst[i] * 255.0;                         // discretize 0-255
                }
            } break;
            case SantosOriginal: {
                const cv::Vec3b* src = in->value.ptr<cv::Vec3b>();
                float* dst = out->value.ptr<float>();
                uchar* mask = out_mask->value.ptr<uchar>();
                const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

                const double theta = M_PI * alpha_;
                const double ct = std::cos(theta);
                const double st = std::sin(theta);

                const static cv::Vec3f v1(1.0 / std::sqrt(2.0), -1.0 / std::sqrt(2.0), 0.0);
                const static cv::Vec3f v2(1.0 / std::sqrt(6.0), 1.0 / std::sqrt(6.0),
                                          -2.0 / std::sqrt(6.0));  //// Entropy Minimization for Shadow Removal

                // double min = 0.0;
                double max = std::log(1.0 / std::sqrt(1.0 / 255.0)) / std::sqrt(2.0);

                for (std::size_t i = 0; i < size; ++i) {
                    const cv::Vec3b& p = src[i];
                    if (p[0] == 0 && p[1] == 0 && p[2] == 0) {
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
            } break;

            case Santos: {
                const cv::Vec3b* src = in->value.ptr<cv::Vec3b>();
                float* dst = out->value.ptr<float>();
                uchar* mask = out_mask->value.ptr<uchar>();
                const std::size_t size = static_cast<std::size_t>(out->value.cols * out->value.rows);

                const double theta = M_PI * alpha_;
                const double ct = std::cos(theta);
                const double st = std::sin(theta);
                const static cv::Vec3f v1(1.0 / std::sqrt(2.0), -1.0 / std::sqrt(2.0), 0.0);
                const static cv::Vec3f v2(1.0 / std::sqrt(6.0), 1.0 / std::sqrt(6.0),
                                          -2.0 / std::sqrt(6.0));  //// Entropy Minimization for Shadow Removal

                // p_M = cbrt(pr*pg*pb)                    max=1, min=1/255
                // chi_r,g,b = log(p_r,g,b/p_M)            max=log(255), min=log(1)
                // psi_1,2 = U x chi_r,g,b                 psi_1:max/min = +/-
                // (1/sqrt(2)*log(255)), psi_2:max/min = +/- (2/sqrt(6)*log(255)) I =
                // psi_1*cos(theta)+psi_2*sin(theta)   max/min = +/-
                // (log(255)*(1/2+1/sqrt(3))

                double max = std::log(255.0) * (0.5 + (1.0 / std::sqrt(3)));
                double min = -max;

                auto get_thresholded = [&](int i) -> std::pair<double, double> {
                    if (use_threshold) {
                        if (dst[i] < threshold_min_max.first) {
                            return std::make_pair(threshold_min_max.first, 2);
                        } else if (dst[i] > threshold_min_max.second) {
                            return std::make_pair(threshold_min_max.second, 3);
                        }
                    }
                    return std::make_pair(dst[i], mask[i]);
                };

                for (std::size_t i = 0; i < size; ++i) {
                    const cv::Vec3b& p = src[i];
                    if (p[0] == 0 || p[1] == 0 || p[2] == 0) {
                        mask[i] = 1;
                        //                    dst[i] = 128;
                        dst[i] = (p[2] > p[1] || p[2] > p[0]) ? 1 : 0;
                        continue;
                    }

                    cv::Vec3f c(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0);

                    const double rho = std::pow((c[0] * c[1] * c[2]), 1.0 / 3);
                    c /= rho;
                    c[0] = std::log(c[0]);
                    c[1] = std::log(c[1]);
                    c[2] = std::log(c[2]);
                    const cv::Vec2f psi(c.ddot(v1), c.ddot(v2));
                    dst[i] = ((psi[0] * ct + psi[1] * st) - min) / (max - min);  // normalize

                    std::tie(dst[i], mask[i]) = get_thresholded(i);  // threshold min/max
                                                                     // vals

                    dst[i] = dst[i] * 255.0;  // discretize 0-255
                }
            } break;
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
        output_mask_ = node_modifier.addOutput<CvMatMessage>("label mask");
    }

    virtual void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::ParameterFactory::declareRange("alpha", 0.0, 1.0, 1.5, 0.001), alpha_);
        std::map<std::string, int> methods = { { "Newman", (int)Newman }, { "NewmanOriginal", (int)NewmanOriginal }, { "Santos", (int)Santos }, { "SantosOriginal", (int)SantosOriginal } };

        parameters.addParameter(param::ParameterFactory::declareParameterSet<int>("method", methods, Newman), method_);

        param::Parameter::Ptr method = param::factory::declareBool("threshold", param::ParameterDescription("Apply min/max thresholding."), true);

        parameters.addParameter(method, std::bind(&BGRToIlluminationInvariant::submit, this));

        std::function<bool()> k_cond = (std::bind(&param::Parameter::as<bool>, method.get()));
        parameters.addConditionalParameter(param::factory::declareInterval("threshold range", param::ParameterDescription("Valid and acceptable range"), 0, 100, 0, 100, 1), k_cond,
                                           std::bind(&BGRToIlluminationInvariant::submit, this));

        parameters.setParameter("threshold range", std::pair<int, int>(5, 95));
    }
    void submit()
    {
    }

protected:
    csapex::Output* output_;
    csapex::Output* output_mask_;

    enum Type
    {
        Newman,
        NewmanOriginal,
        Santos,
        SantosOriginal
    };

    csapex::Input* input_;
    double alpha_;
    int method_;
    bool recover_bgr_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::BGRToIlluminationInvariant, csapex::Node)
