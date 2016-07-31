/// HEADER
#include "svm_array.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::SVMArray, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

template<typename T>
inline std::string toString(const T value)
{
    std::stringstream ss;
    ss << value;
    return ss.str();
}

SVMArray::SVMArray() :
    loaded_(false)
{
}

void SVMArray::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage::Ptr>("responses");

    reload_ = node_modifier.addSlot("Reload", std::bind(&SVMArray::load, this));

}

void SVMArray::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declarePath("svm array path",
                                                      csapex::param::ParameterDescription("Path to a saved svm."),
                                                      true,
                                                      "",
                                                      "*.yaml *.tar.gz"),
                 std::bind(&SVMArray::load, this));

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

void SVMArray::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<CvMatMessage::Ptr>> output(new std::vector<CvMatMessage::Ptr>);

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

    /// TODO : OMP?

    std::size_t size = input->size();
    output->resize(size);
    for(std::size_t i = 0 ; i < size ; ++i)
    {
        cv::Mat sample(input->at(i).value);
        CvMatMessage::Ptr &result_msg = output->at(i);
        result_msg.reset(new CvMatMessage(enc::unknown, 0));
        cv::Mat result = result_msg->value;
        result = svm_responses_.clone();
        for(std::size_t j = 0 ; j < svms_size_ ; ++j) {
            SVMPtr svm = svms_.at(j);
            if (compute_label)
                result.at<float>(1,j) = svm->predict(sample);
            else
            {
                const float response = svm->predict(sample, true);
                result.at<float>(1,j) = comparator(response) ? POSITIVE : NEGATIVE;
            }
        }
    }

    msg::publish<GenericVectorMessage, CvMatMessage::Ptr>(out_, output);
}

void SVMArray::load()
{
    std::string path = readParameter<std::string>("svm array path");
    if(path == "")
        return;

    const static std::string prefix = "svm_";
    cv::FileStorage fs(path, cv::FileStorage::READ);
    std::vector<int> labels;
    fs["labels"] >> labels;

    if(labels.empty())
        throw std::runtime_error("File entry 'labels' may not be empty!");

    /// get svms for labels
    svm_responses_ = cv::Mat(labels.size(), 2, CV_32FC1, cv::Scalar());
    for(std::size_t i = 0 ; i < labels.size() ; ++i) {
        std::string label = prefix + toString(labels.at(i));
        SVMPtr svm(new cv::SVM);
        svm->read(fs.fs, (CvFileNode*) fs[label].node);
        svms_.push_back(svm);
        svm_responses_.at<float>(i, 0) = labels.at(i);
    }
    svms_size_ = svms_.size();
    loaded_ = true;
}
