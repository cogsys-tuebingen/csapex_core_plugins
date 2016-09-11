/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <functional>


namespace csapex {

using namespace csapex;
using namespace connection_types;

class ScaleROIs : public csapex::Node
{
public:
    ScaleROIs()
    {

    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_rois_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        in_img_   = node_modifier.addOptionalInput<CvMatMessage>("Cap to Image");
        out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("Scaled ROIs");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(csapex::param::ParameterFactory::declareRange("percent x", 1.0, 400.0, 100.0, 1.0),
                                scales_[0]);
        parameters.addParameter(csapex::param::ParameterFactory::declareRange("percent y", 1.0, 400.0, 100.0, 1.0),
                                scales_[1]);
    }

    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> in_rois =
                msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

        std::shared_ptr<std::vector<RoiMessage>> out_rois(new std::vector<RoiMessage>);
        out_rois->assign(in_rois->begin(), in_rois->end());

        if(msg::hasMessage(in_img_)) {
            CvMatMessage::ConstPtr in_img =
                    msg::getMessage<CvMatMessage>(in_img_);

            const cv::Rect image_bounds(0,0, in_img->value.cols, in_img->value.rows);
            for(RoiMessage &r : *out_rois) {
                cv::Rect rect = r.value.rect();
                int height = rect.height * scales_[0] / 100.0;
                int width  = rect.width  * scales_[1] / 100.0;
                int off_x  = std::floor((rect.width - width) / 2.0);
                int off_y  = std::floor((rect.height - height) / 2.0);
                rect.height = height;
                rect.width = width;
                rect.x += off_x;
                rect.y += off_y;

                r.value.setRect(rect & image_bounds);
            }
        } else {
            for(RoiMessage &r : *out_rois) {
                cv::Rect rect = r.value.rect();
                int height = rect.height * scales_[0] / 100.0;
                int width  = rect.width  * scales_[1] / 100.0;
                int off_x  = std::floor((rect.width - width) / 2.0);
                int off_y  = std::floor((rect.height - height) / 2.0);
                rect.height = height;
                rect.width = width;
                rect.x += off_x;
                rect.y += off_y;

                r.value.setRect(rect);
            }
        }

        msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);
    }

private:
    Input  *in_img_;
    Input  *in_rois_;
    Output *out_rois_;

    cv::Vec2d scales_;
};
}

CSAPEX_REGISTER_CLASS(csapex::ScaleROIs, csapex::Node)
