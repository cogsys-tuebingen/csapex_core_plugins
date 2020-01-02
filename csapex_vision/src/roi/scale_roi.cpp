/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>

/// SYSTEM
#include <functional>

namespace csapex
{
using namespace csapex;
using namespace connection_types;

class ScaleROI : public csapex::Node
{
public:
    ScaleROI()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_roi_ = node_modifier.addInput<RoiMessage>("ROI");
        in_img_ = node_modifier.addOptionalInput<CvMatMessage>("Cap to Image");
        out_roi_ = node_modifier.addOutput<RoiMessage>("Scaled ROI");
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(csapex::param::factory::declareRange("percent x", 1.0, 400.0, 100.0, 1.0), scales_[0]);
        parameters.addParameter(csapex::param::factory::declareRange("percent y", 1.0, 400.0, 100.0, 1.0), scales_[1]);
    }

    virtual void process() override
    {
        RoiMessage::ConstPtr in_roi = msg::getMessage<RoiMessage>(in_roi_);

        RoiMessage::Ptr out_roi(new RoiMessage);
        out_roi->value = in_roi->value;

        cv::Rect rect = out_roi->value.rect();
        int height = rect.height * scales_[1] / 100.0;
        int width = rect.width * scales_[0] / 100.0;
        int off_x = std::floor((rect.width - width) / 2.0);
        int off_y = std::floor((rect.height - height) / 2.0);
        rect.height = height;
        rect.width = width;
        rect.x += off_x;
        rect.y += off_y;

        if (msg::hasMessage(in_img_)) {
            CvMatMessage::ConstPtr in_img = msg::getMessage<CvMatMessage>(in_img_);

            const cv::Rect image_bounds(0, 0, in_img->value.cols, in_img->value.rows);
            rect &= image_bounds;
        }

        out_roi->value.setRect(rect);

        msg::publish(out_roi_, out_roi);
    }

private:
    Input* in_img_;
    Input* in_roi_;
    Output* out_roi_;

    cv::Vec2d scales_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ScaleROI, csapex::Node)
