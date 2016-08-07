/// HEADER
#include "svm.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::SVM, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SVM::SVM() :
    loaded_(false)
{
}

void SVM::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("labelled features");

    reload_ = node_modifier.addSlot("Reload", std::bind(&SVM::load, this));

}

void SVM::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declarePath("svm path",
                                                      csapex::param::ParameterDescription("Path to a saved svm."),
                                                      true,
                                                      "",
                                                      "*.yaml *.tar.gz"),
                 std::bind(&SVM::load, this));

    csapex::param::ParameterPtr param_label = csapex::param::ParameterFactory::declareBool("compute labels",
                                                                                           csapex::param::ParameterDescription("Directly compute labels. 'false' allows manual threshold setting for binary classification"),
                                                                                           true);
    addParameter(param_label);

    std::map<std::string, int> svm_thresh_types = {
        {">", GREATER},
        {"<" , LESS},
        {">=", GREATER_EQUAL},
        {"<=", LESS_EQUAL}
    };
    auto threshold_condition = [param_label]() { return param_label->as<bool>() == false; };
    addConditionalParameter(param::ParameterFactory::declareParameterSet("threshold/type",
                                                                         csapex::param::ParameterDescription("SVM threshold comparison type"),
                                                                         svm_thresh_types,
                                                                         (int) GREATER),
                            threshold_condition);

    addConditionalParameter(csapex::param::ParameterFactory::declareRange("threshold/value",
                                                                          csapex::param::ParameterDescription("SVM threshold for binary classification"),
                                                                          -1000.0,
                                                                          1000.0,
                                                                          0.0,
                                                                          0.01),
                            threshold_condition);
}

void SVM::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);
    output->insert(output->end(), input->begin(), input->end());

    if(!loaded_) {
        throw std::runtime_error("No SVM is loaded!");
    }

    const bool compute_label = readParameter<bool>("compute labels");
    std::function<bool(float)> comparator;

    {
        const float threshold = readParameter<double>("threshold/value");
        const int comparison_type = readParameter<int>("threshold/type");

        switch (comparison_type)
        {
        default:
        case GREATER:
            comparator = [threshold](float value) { return value > threshold; };
            break;
        case GREATER_EQUAL:
            comparator = [threshold](float value) { return value >= threshold; };
            break;
        case LESS:
            comparator = [threshold](float value) { return value < threshold; };
            break;
        case LESS_EQUAL:
            comparator = [threshold](float value) { return value <= threshold; };
            break;
        }
    }

    std::size_t size = input->size();
    for(std::size_t i = 0 ; i < size ; ++i)
    {
        cv::Mat sample(output->at(i).value);
        if (compute_label)
            output->at(i).classification = svm_.predict(sample);
        else
        {
            const float response = svm_.predict(sample, true);
            output->at(i).classification = comparator(response) ? 1 : 0;
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

void SVM::load()
{
    std::string path = readParameter<std::string>("svm path");
    if(path == "")
        return;

    svm_.load(path.c_str(), "svm");
    loaded_ = true;
}
