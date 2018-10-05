
/// PROJECT
#include "feature_classification_mapping.hpp"
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
/// SYSTEM
#include <fstream>
#include <yaml-cpp/yaml.h>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class AssignFeatureClassificationRunningLabels : public Node
{
public:
    AssignFeatureClassificationRunningLabels()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, FeaturesMessage>("Input");
        out_ = modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Output");
        out_classes_ = modifier.addOutput<int>("number of classes");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareTrigger("clear"), [this](param::Parameter*) { clear(); });

        params.addParameter(param::factory::declareTrigger("save"), [this](param::Parameter*) { save(); });

        params.addParameter(param::factory::declareFileOutputPath("file", "/tmp/feature_mapping"), path_);
    }

    void process() override
    {
        std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
        std::shared_ptr<std::vector<FeaturesMessage>> out(new std::vector<FeaturesMessage>);
        for (auto feature : *input) {
            FeaturesMessage relabeled = feature;
            int label = feature.classification;
            auto it = std::find(classifications_.begin(), classifications_.end(), label);
            if (it == classifications_.end()) {
                classifications_.push_back(label);
                relabeled.classification = classifications_.size() - 1;
            } else {
                relabeled.classification = it - classifications_.begin();
            }
            out->push_back(relabeled);
        }

        int n_classes = classifications_.size();
        msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out);
        msg::publish(out_classes_, n_classes);
    }

    void clear()
    {
        classifications_.clear();
    }

    void save()
    {
        FeatureClassificationMapping mapping;
        std::size_t new_label = 0;
        for (auto val : classifications_) {
            mapping.addRule(val, new_label);
            ++new_label;
        }
        mapping.save(path_);
    }

private:
    Input* in_;
    Output* out_;
    Output* out_classes_;
    std::string path_;
    std::vector<int> classifications_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::AssignFeatureClassificationRunningLabels, csapex::Node)
