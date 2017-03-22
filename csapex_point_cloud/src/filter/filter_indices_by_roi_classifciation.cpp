#include <csapex/model/node.h>

#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex_opencv/roi_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/indeces_message.h>

#include <set>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace csapex
{
using namespace connection_types;

class FilterIndicesByRoiClassification : public Node
{
public:
    void setupParameters(csapex::Parameterizable& parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareValue<std::string>("classes",
                                                                                   param::ParameterDescription("Comma separated list of allowed classification labels"),
                                                                                   "0,1"),
                                std::bind(&FilterIndicesByRoiClassification::updateLabels, this));
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_indices_ = node_modifier.addMultiInput<GenericVectorMessage, pcl::PointIndices>("indices");
        in_rois_     = node_modifier.addInput<GenericVectorMessage, RoiMessage>("rois");
        out_indices_    = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("filtered rois");
    }

    void process() override
    {
        std::shared_ptr<std::vector<pcl::PointIndices> const> in_indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);
        std::shared_ptr<std::vector<RoiMessage> const>        in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
        std::shared_ptr<std::vector<pcl::PointIndices>>       out_indices(new std::vector<pcl::PointIndices>);

        if(in_indices->size() != in_rois->size()) {
            throw std::runtime_error("Indices and roi counts do not match!");
        }

        const std::size_t size = in_indices->size();
        for(std::size_t i = 0 ; i < size ; ++i) {
            if(labels_.find(in_rois->at(i).value.classification()) != labels_.end()) {
                out_indices->emplace_back(in_indices->at(i));
            }
        }

        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_indices_, out_indices);
    }


private:
    csapex::Input* in_indices_;
    csapex::Input* in_rois_;
    csapex::Output* out_indices_;
    std::set<int> labels_;

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

};
}

CSAPEX_REGISTER_CLASS(csapex::FilterIndicesByRoiClassification, csapex::Node)


