#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>

#include <set>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace csapex::connection_types;

namespace csapex
{
class FilterROIsByClassification : public csapex::Node
{
public:
    void setupParameters(csapex::Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareValue<std::string>("classes",
                                                                                   param::ParameterDescription("Comma separated list of allowed classification labels"),
                                                                                   "0,1"),
                                std::bind(&FilterROIsByClassification::updateLabels, this));
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_rois_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("rois");
        out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("filtered rois");
    }

    void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
        std::shared_ptr<std::vector<RoiMessage>> out_rois = std::make_shared<std::vector<RoiMessage>>();

        for (const RoiMessage& roi_msg : *in_rois)
        {
            if (labels_.find(roi_msg.value.classification()) != labels_.end())
                out_rois->emplace_back(roi_msg);
        }

        msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);

    }

    void updateLabels()
    {
        std::string label_string = readParameter<std::string>("classes");

        labels_.clear();

        std::vector<std::string> labels;
        boost::algorithm::split(labels, label_string, boost::is_any_of(","));
        try
        {
            std::transform(labels.begin(), labels.end(), std::inserter(labels_, labels_.begin()),
                           [](const std::string& text)
            {
                return boost::lexical_cast<int>(text);
            });
        }
        catch (boost::bad_lexical_cast ex)
        {
            node_modifier_->setError("Invalid label in sequence, must be an integer");
            return;
        }
        node_modifier_->setNoError();
    }

private:
    csapex::Input* in_rois_;
    csapex::Output* out_rois_;

    std::set<int> labels_;
};
}

CSAPEX_REGISTER_CLASS(csapex::FilterROIsByClassification, csapex::Node)
