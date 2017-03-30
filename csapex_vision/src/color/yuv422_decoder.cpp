/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex::connection_types;


namespace csapex
{

class YUV422Decoder : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<CvMatMessage>("2-channel YUV422 Image");
        out_ = modifier.addOutput<CvMatMessage>("3-channel YUV422 Image");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        CvMatMessage::ConstPtr img_in = msg::getMessage<CvMatMessage>(in_);
        cv::Mat img = img_in->value;

        if(img.channels() != 2) {
            throw std::logic_error("image has to be 2-channel.");
        }

        std::vector<cv::Mat> in_channels;
        cv::split(img, in_channels);
        const cv::Mat& uv_combined = in_channels.at(0);
        const cv::Mat& y = in_channels.at(1);

        std::vector<cv::Mat> out_channels;

        cv::Mat uv = uv_combined.reshape(2);

        std::vector<cv::Mat> uv_split;
        cv::split(uv, uv_split);
        cv::Mat u = uv_split.front();
        cv::Mat v = uv_split.back();

        cv::resize(u, u, cv::Size(), 2.0, 1.0, cv::INTER_CUBIC);
        cv::resize(v, v, cv::Size(), 2.0, 1.0, cv::INTER_CUBIC);

        out_channels.push_back(y);
        out_channels.push_back(v);
        out_channels.push_back(u);

        CvMatMessage::Ptr img_out(new CvMatMessage(img_in->getEncoding(), img_in->frame_id, img_in->stamp_micro_seconds));
        cv::Mat& out = img_out->value;
//        out = cv::Mat(img.rows, img.cols, CV_8UC3, cv::Scalar::all(128));

        cv::merge(out_channels, out);

        msg::publish(out_, img_out);
    }

private:
    Input* in_;
    Output* out_;
};

}

CSAPEX_REGISTER_CLASS(csapex::YUV422Decoder, csapex::Node)

