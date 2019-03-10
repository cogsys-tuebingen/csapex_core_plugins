
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class Scale16BitTo8Bit : public Node
{
public:
    Scale16BitTo8Bit()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<CvMatMessage>("16 bit image");
        out_ = modifier.addOutput<CvMatMessage>("8 bit image");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("significant bits", 8, 16, 12, 1), significant_);
    }

    void process() override
    {
        CvMatMessage::ConstPtr img_msg = msg::getMessage<CvMatMessage>(in_);
        const cv::Mat& img = img_msg->value;
        if (img.depth() == 8) {
            msg::publish(out_, img_msg);
            return;
        }

        CvMatMessage::Ptr img_out(new CvMatMessage(img_msg->getEncoding(), img_msg->frame_id,
                                                   img_msg->stamp_micro_seconds));

        cv::Mat& target = img_out->value;
        double scale = std::pow(2.0, significant_) / static_cast<double>(std::pow(2, 16));
        img.convertTo(target, CV_MAKE_TYPE(8, img.channels()), scale);

        msg::publish(out_, img_out);
    }

private:
    int significant_;

    Input* in_;
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::Scale16BitTo8Bit, csapex::Node)
