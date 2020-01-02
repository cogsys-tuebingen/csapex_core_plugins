/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class Sample : public Node
{
public:
    virtual void process() override
    {
        CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(in_);
        CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

        switch (in->value.type() & 7) {
            case CV_8U:
                sample<uchar>(in->value, out->value);
                break;
            case CV_8S:
                sample<char>(in->value, out->value);
                break;
            case CV_16U:
                sample<ushort>(in->value, out->value);
                break;
            case CV_16S:
                sample<short>(in->value, out->value);
                break;
            case CV_32S:
                sample<int>(in->value, out->value);
                break;
            case CV_32F:
                sample<float>(in->value, out->value);
                break;
            case CV_64F:
                sample<double>(in->value, out->value);
                break;
            default:
                throw std::runtime_error("Unknown opencv matrix type!");
        }

        msg::publish(out_, out);
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addInput<CvMatMessage>("Matrix");
        out_ = node_modifier.addOutput<CvMatMessage>("Sampled");
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareRange("step x", 1, 100, 2, 1), step_x_);
        parameters.addParameter(param::factory::declareRange("step y", 1, 100, 2, 1), step_y_);
    }

private:
    Input* in_;
    Output* out_;

    int step_x_;
    int step_y_;

    template <typename T>
    void sample(const cv::Mat& src, cv::Mat& dst)
    {
        dst = cv::Mat(src.rows / step_y_, src.cols / step_x_, src.type(), cv::Scalar());

        const T* src_ptr = src.ptr<T>();
        T* dst_ptr = dst.ptr<T>();
        const int channels = dst.channels();
        int i_ = 0;
        for (int i = 0; i < dst.rows; ++i, i_ += step_y_) {
            int j_ = 0;
            for (int j = 0; j < dst.cols; ++j, j_ += step_x_) {
                for (int c = 0; c < channels; ++c) {
                    dst_ptr[(i * dst.cols + j) * channels + c] = src_ptr[(i_ * src.cols + j_) * channels + c];
                }
            }
        }
    }
};

}  // namespace csapex
CSAPEX_REGISTER_CLASS(csapex::Sample, csapex::Node)
