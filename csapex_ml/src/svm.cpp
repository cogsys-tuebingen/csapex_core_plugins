/// HEADER
#include "svm.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>

CSAPEX_REGISTER_CLASS(csapex::SVM, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SVM::SVM() : loaded_(false)
{
}

void SVM::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("labelled features");

    reload_ = node_modifier.addSlot("Reload", std::bind(&SVM::reload, this));
}

void SVM::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declarePath("/svm/path", csapex::param::ParameterDescription("Path to a saved svm."), true, "", "*.yaml *.tar.gz *.yaml.gz"), path_);

    csapex::param::ParameterPtr param_label = csapex::param::factory::declareBool("/svm/compute_labels",
                                                                                  csapex::param::ParameterDescription("Directly compute labels. 'false' allows manual threshold setting "
                                                                                                                      "for binary classification"),
                                                                                  true);
    parameters.addParameter(param_label);

    std::map<std::string, int> svm_thresh_types = { { ">", GREATER }, { "<", LESS }, { ">=", GREATER_EQUAL }, { "<=", LESS_EQUAL } };
    auto threshold_condition = [param_label]() { return param_label->as<bool>() == false; };
    parameters.addConditionalParameter(param::factory::declareParameterSet("/svm/threshold/type", csapex::param::ParameterDescription("SVM threshold comparison type"), svm_thresh_types, (int)GREATER),
                                       threshold_condition);

    parameters.addConditionalParameter(
        csapex::param::factory::declareRange("/svm/threshold/value", csapex::param::ParameterDescription("SVM threshold for binary classification"), -100.0, 100.0, 0.0, 0.001), threshold_condition);
}

void SVM::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<FeaturesMessage>> output(new std::vector<FeaturesMessage>);
    output->insert(output->end(), input->begin(), input->end());

    if (!loaded_) {
        if (path_ != "") {
#if CV_MAJOR_VERSION == 2
            svm_.load(path_.c_str(), "svm");
#elif CV_MAJOR_VERSION >= 3
            svm_ = cv::ml::SVM::load(path_.c_str());
#endif
            loaded_ = true;
        } else {
            throw std::runtime_error("SVM couldn't be loaded!");
        }
    }

    const bool compute_label = readParameter<bool>("/svm/compute_labels");
    std::function<bool(float)> comparator;

    {
        const float threshold = readParameter<double>("/svm/threshold/value");
        const int comparison_type = readParameter<int>("/svm/threshold/type");

        switch (comparison_type) {
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
    for (std::size_t i = 0; i < size; ++i) {
        cv::Mat sample(output->at(i).value);
        if (compute_label) {
#if CV_MAJOR_VERSION == 2
            output->at(i).classification = svm_.predict(sample);
#elif CV_MAJOR_VERSION >= 3
            cv::transpose(sample, sample);
            output->at(i).classification = svm_->predict(sample);
#endif
        } else {
#if CV_MAJOR_VERSION == 2
            const float response = svm_.predict(sample, true);
#elif CV_MAJOR_VERSION >= 3
            cv::transpose(sample, sample);
            const float response = svm_->predict(sample, cv::noArray(), cv::ml::StatModel::RAW_OUTPUT);
#endif
            output->at(i).classification = comparator(response) ? 1 : 0;
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

void SVM::reload()
{
    loaded_ = false;
}
