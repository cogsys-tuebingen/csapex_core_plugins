
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/signal/slot.h>
#include <csapex_ml/features_message.h>
#include "feature_classification_mapping.hpp"

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class DecodeFeatureClassification : public Node
{
public:
    DecodeFeatureClassification()
        : loaded_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, FeaturesMessage>("Input");
        out_ = modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Output");
        reload_ = modifier.addSlot("Reload", std::bind(&DecodeFeatureClassification::reloadConfig, this));
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareFileInputPath("saved_mapping",""),
                            [this](param::Parameter* p){
            std::string path = p->as<std::string>();
            if(path != path_){
                path_ = path;
                reloadConfig();
            }
        });


    }

    void process() override
    {
        if(!loaded_){
            if(path_ != "") {
                loaded_ = mapping_.load(path_);
                if(!loaded_){
                    throw std::runtime_error("Mapping couldn't be loaded!");
                }
            }
            return;
        }
        std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
        std::shared_ptr<std::vector<FeaturesMessage>> out(new std::vector<FeaturesMessage>);
        for(auto feature : *input){
            FeaturesMessage relabeled = feature;
            bool success = mapping_.inverse(feature, relabeled);
            if(!success){
                throw std::runtime_error("Classification does not match any rule!");
            }
            out->push_back(relabeled);
        }

        msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out);
    }
private:
    void reloadConfig()
    {
        loaded_ = false;
    }

private:
    bool loaded_;
    Input* in_;
    Output* out_;
    Slot* reload_;
    std::string path_;
    FeatureClassificationMapping mapping_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::DecodeFeatureClassification, csapex::Node)

