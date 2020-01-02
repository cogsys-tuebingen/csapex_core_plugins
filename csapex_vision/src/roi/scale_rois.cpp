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
enum Modes
{
    CENTERED,
    LINEAR
};

using namespace csapex;
using namespace connection_types;

class ScaleROIs : public csapex::Node
{
public:
    ScaleROIs() : mode_(0)
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        in_img_ = node_modifier.addOptionalInput<CvMatMessage>("Cap to Image");
        out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("Scaled ROIs");
    }

    void setupParameters(Parameterizable& parameters) override
    {
        std::map<std::string, int> modes = { { "centered", CENTERED }, { "linear", LINEAR } };

        parameters.addParameter(csapex::param::factory::declareParameterSet("mode", modes, 0), mode_);

        parameters.addParameter(csapex::param::factory::declareRange("percent x", 1.0, 400.0, 100.0, 1.0), scales_[0]);
        parameters.addParameter(csapex::param::factory::declareRange("percent y", 1.0, 400.0, 100.0, 1.0), scales_[1]);
    }

    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

        std::shared_ptr<std::vector<RoiMessage>> out_rois(new std::vector<RoiMessage>);
        out_rois->assign(in_rois->begin(), in_rois->end());

        if (msg::hasMessage(in_img_)) {
            CvMatMessage::ConstPtr in_img = msg::getMessage<CvMatMessage>(in_img_);

            const cv::Rect image_bounds(0, 0, in_img->value.cols, in_img->value.rows);
            for (RoiMessage& r : *out_rois) {
                cv::Rect rect = r.value.rect();
                scale(rect);

                r.value.setRect(rect & image_bounds);
            }
        } else {
            for (RoiMessage& r : *out_rois) {
                cv::Rect rect = r.value.rect();
                scale(rect);

                r.value.setRect(rect);
            }
        }

        msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);
    }

private:
    void scale(cv::Rect& rect)
    {
        int height = rect.height * scales_[1] / 100.0;
        int width = rect.width * scales_[0] / 100.0;

        switch (mode_) {
            case CENTERED:
                rect.x += (rect.width - width) / 2;
                rect.y += (rect.height - height) / 2;
                break;
            case LINEAR:
                rect.x *= scales_[0] / 100.0;
                rect.y *= scales_[1] / 100.0;
                break;
            default:
                break;
        }

        rect.height = height;
        rect.width = width;
    }

private:
    Input* in_img_;
    Input* in_rois_;
    Output* out_rois_;

    int mode_;

    cv::Vec2d scales_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ScaleROIs, csapex::Node)
