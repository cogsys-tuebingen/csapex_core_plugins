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
#include <fstream>

namespace csapex {

using namespace connection_types;

class ROIHitRate : public csapex::Node
{
public:
    enum OverlapMode {OVERLAP_IOU, OVERLAP_MAX};
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
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
        std::map<std::string, int> overlap_mode_types = {
                {"MAX", OVERLAP_MAX},
                {"IOU", OVERLAP_IOU},
        };
        parameters.addParameter(param::factory::declareParameterSet("overlap_mode",
                                                                             overlap_mode_types,
                                                                             static_cast<int>(OVERLAP_MAX)),
                                reinterpret_cast<int&>(overlap_mode_));
        parameters.addParameter(param::factory::declareRange("min_overlap", 0.0, 1.0, 0.7,0.01),
                                min_overlap_);
        parameters.addParameter(param::factory::declareFileOutputPath("statistic path",
                                                                               "",
                                                                               "*.txt"),
                                path_of_statistic_);

        parameters.addParameter(param::factory::declareTrigger("reset"),
                                std::bind(&ROIHitRate::reset, this));
        parameters.addParameter(param::factory::declareTrigger("save"),
                                std::bind(&ROIHitRate::save, this));

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
                double overlap = 0.0;
                switch (overlap_mode_) {
                    default:
                    case OVERLAP_MAX:
                        overlap = overlap_max(roi_pred.value.rect(), roi_gt.value.rect());
                        break;
                    case OVERLAP_IOU:
                        overlap = overlap_iou(roi_pred.value.rect(), roi_gt.value.rect());
                        break;
                }
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
    OverlapMode overlap_mode_;
    double      min_overlap_;
    std::string path_of_statistic_;

    inline double overlap_max(const cv::Rect &prediction,
                              const cv::Rect &groundtruth)
    {
        return (prediction & groundtruth).area() / (double) std::max(prediction.area(),
                                                                     groundtruth.area());
    }

    inline double overlap_iou(const cv::Rect &prediction,
                              const cv::Rect &groundtruth)
    {
        return (prediction & groundtruth).area() / (double) (prediction | groundtruth).area();
    }

    void save()
    {
        std::ofstream out(path_of_statistic_);
        if(!out.is_open())
            throw std::runtime_error("Cannot open path '" + path_of_statistic_ + "'!");

        out << "total : hit : pct" << std::endl;
        out << ground_truth_rois_ << " : "
            << ground_truth_rois_hit_ << " : "
            << (ground_truth_rois_hit_ / (double) ground_truth_rois_) << std::endl;
        out.close();
    }

    void reset()
    {
        ground_truth_rois_ = 0;
        ground_truth_rois_hit_ = 0;
    }

};
}
CSAPEX_REGISTER_CLASS(csapex::ROIHitRate, csapex::Node)
