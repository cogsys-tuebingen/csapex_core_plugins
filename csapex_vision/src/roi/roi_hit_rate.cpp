/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_ml/features_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex_evaluation/confusion_matrix_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;

class ROIHitRate : public csapex::Node
{
public:
    typedef std::shared_ptr<std::vector<RoiMessage> const> RoiMessagesPtr;

    ROIHitRate() :
        ground_truth_rois_(0),
        ground_truth_rois_hit_(0)
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_gt_         = node_modifier.addInput<GenericVectorMessage, RoiMessage>("Ground Truth");
        input_prediction_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("Predicted");
        output_hit_rate_ = node_modifier.addOutput<std::string>("Hitrate");

        node_modifier.addSlot("Reset",
                              std::bind(&ROIHitRate::reset, this));
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                                std::bind(&ROIHitRate::reset, this));
        parameters.addParameter(param::ParameterFactory::declareRange("min_overlap", 0.0, 1.0, 0.7,0.01),
                                min_overlap_);

    }
    virtual void process() override
    {
        RoiMessagesPtr input_gt =
                msg::getMessage<GenericVectorMessage, RoiMessage>(input_gt_);
        RoiMessagesPtr input_prediction =
                msg::getMessage<GenericVectorMessage, RoiMessage>(input_prediction_);


        const std::size_t size_gt   = input_gt->size();
        const std::size_t size_pred = input_prediction->size();
        std::vector<char> mask_pred(size_pred, 1);
        for(std::size_t i = 0 ; i < size_gt ; ++i) {
            const RoiMessage &roi_gt = input_gt->at(i);
            std::size_t  max_idx = 0;
            double       max_overlap = std::numeric_limits<double>::lowest();
            for(std::size_t j = 0 ; j < size_pred ; ++j) {
                if(mask_pred[j] == 0)
                    continue;

                const RoiMessage &roi_pred = input_prediction->at(j);
                const double overlap = ROIHitRate::overlap(roi_pred.value.rect(),
                                                           roi_gt.value.rect());
                if(overlap > max_overlap) {
                    max_idx = j;
                    max_overlap = overlap;
                }
            }

            if(max_overlap > min_overlap_) {
                ++ground_truth_rois_hit_;
                mask_pred[max_idx] = 0;
            }
            ++ground_truth_rois_;
        }

        if(ground_truth_rois_ > 0)
            msg::publish(output_hit_rate_, std::to_string(ground_truth_rois_hit_ / (double) ground_truth_rois_));
    }

private:
    Input* input_gt_;
    Input* input_prediction_;

    Output* output_hit_rate_;

    std::size_t ground_truth_rois_;
    std::size_t ground_truth_rois_hit_;
    double      min_overlap_;

    inline double overlap(const cv::Rect &prediction,
                          const cv::Rect &groundtruth)
    {
        return (prediction & groundtruth).area() / (double) std::max(prediction.area(),
                                                                     groundtruth.area());
    }

    void reset()
    {
        ground_truth_rois_ = 0;
        ground_truth_rois_hit_ = 0;
    }

};
}
CSAPEX_REGISTER_CLASS(csapex::ROIHitRate, csapex::Node)
