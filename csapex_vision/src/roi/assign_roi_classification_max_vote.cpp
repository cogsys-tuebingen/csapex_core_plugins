/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <csapex_opencv/roi_message.h>

namespace csapex
{
using namespace connection_types;

class AssignROIClassificationMaxVote : public csapex::Node
{
public:
    AssignROIClassificationMaxVote()
    {
    }

    void setupParameters(Parameterizable& parameter)
    {
    }

    void setup(NodeModifier& node_modifier)
    {
        in_votes_ = node_modifier.addInput<GenericVectorMessage, std::vector<double>>("class votes");
        in_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("rois");
        out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("features");
    }

    void process()
    {
        std::shared_ptr<std::vector<RoiMessage> const> rois_in = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
        std::shared_ptr<std::vector<std::vector<double>> const> votes_in = msg::getMessage<GenericVectorMessage, std::vector<double>>(in_votes_);
        std::shared_ptr<std::vector<RoiMessage>> rois_out = std::make_shared<std::vector<RoiMessage>>();

        if (rois_in->size() != votes_in->size())
            throw std::runtime_error("Not the same amount of feature and roi messages!");

        rois_out->assign(rois_in->begin(), rois_in->end());

        const std::size_t size = rois_out->size();
        for (std::size_t i = 0; i < size; ++i) {
            const std::vector<double>& votes = votes_in->at(i);
            RoiMessage& roi_out = rois_out->at(i);

            const auto max_vote = std::max_element(votes.begin(), votes.end(), std::less<double>());
            const std::size_t classification = std::distance(votes.begin(), max_vote);
            roi_out.value.setClassification(classification);

            if (roi_out.value.classification() > 0)
                roi_out.value.setColor(cv::Scalar(0, 255, 0));
            else
                roi_out.value.setColor(cv::Scalar(0, 0, 255));
        }

        msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, rois_out);
    }

private:
    csapex::Input* in_votes_;
    csapex::Input* in_rois_;
    csapex::Output* out_rois_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::AssignROIClassificationMaxVote, csapex::Node)
