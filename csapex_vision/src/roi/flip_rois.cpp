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

class FlipROIs : public csapex::Node
{
public:
    FlipROIs()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_img_ = node_modifier.addInput<CvMatMessage>("Image");
        in_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("Flipped ROIs");
    }

    void setupParameters(Parameterizable& parameters) override
    {
        std::map<std::string, int> types = { { "v", 0 }, { "h", 1 }, { "+90", 2 }, { "-90", 3 }, { "v+h", -1 } };
        parameters.addParameter(csapex::param::factory::declareParameterSet("type", types, -1), mode_);
    }

    virtual void process() override
    {
        CvMatMessage::ConstPtr in_img = msg::getMessage<CvMatMessage>(in_img_);

        std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

        std::shared_ptr<std::vector<RoiMessage>> out_rois(new std::vector<RoiMessage>);
        out_rois->assign(in_rois->begin(), in_rois->end());

        const cv::Mat& img = in_img->value;
        std::function<void(cv::Rect & roi)> modifier;
        switch (mode_) {
            case -1:
                modifier = [img](cv::Rect& rect) {
                    /// flip row and col index
                    rect.y = img.rows - rect.y - rect.height;
                    rect.x = img.cols - rect.x - rect.width;
                };
                break;
            case 0:
                modifier = [img](cv::Rect& rect) {
                    /// flip row index
                    rect.y = img.rows - rect.y - rect.height;
                };
                break;
            case 1:
                modifier = [img](cv::Rect& rect) {
                    /// flip col index
                    rect.x = img.cols - rect.x - rect.width;
                };
                break;
            case 2:
                modifier = [img](cv::Rect& rect) {
                    /// rotate +90
                    std::swap(rect.width, rect.height);
                    rect.x = img.rows - rect.y - rect.height;
                };
                break;
            case 3:
                modifier = [img](cv::Rect& rect) {
                    /// rotate -90
                    std::swap(rect.width, rect.height);
                    rect.y = img.cols - rect.x - rect.width;
                };
                break;
        }

        for (RoiMessage& roi : *out_rois) {
            cv::Rect r = roi.value.rect();
            modifier(r);
            roi.value.setRect(r);
        }
        msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);
    }

private:
    Input* in_img_;
    Input* in_rois_;
    Output* out_rois_;

    int mode_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::FlipROIs, csapex::Node)
