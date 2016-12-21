/// HEADER
#include "svm_ensemble.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::SVMEnsemble, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace impl {
template<typename T>
inline std::string str(const T value)
{
    std::stringstream ss;
    ss << value;
    return ss.str();
}

template<typename T>
inline std::size_t digits(const T number)
{
    std::size_t count = 0;
    T absolute = std::abs(number);
    absolute -= std::floor(absolute);
    while(absolute > 0.0) {
        absolute *= 10;
        ++count;
        absolute -= std::floor(absolute);
    }
    return std::min(count, (std::size_t) 15);
}
}

SVMEnsemble::SVMEnsemble() :
    loaded_(false)
{
}

void SVMEnsemble::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage::ConstPtr>("responses");

    reload_ = node_modifier.addSlot("Reload", std::bind(&SVMEnsemble::load, this) /*[this](){load();}*/);

}

void SVMEnsemble::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declarePath("svm array path",
                                                                         csapex::param::ParameterDescription("Path to a saved svm."),
                                                                         true,
                                                                         "",
                                                                         "*.yaml *.tar.gz"),
                                                                         std::bind(&SVMEnsemble::load, this));

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

    addParameter(param::ParameterFactory::declareParameterSet("threshold/type",
                                                              csapex::param::ParameterDescription("SVM threshold comparison type"),
                                                              svm_thresh_types,
                                                              (int) GREATER));
}

void SVMEnsemble::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<CvMatMessage::ConstPtr>> output(new std::vector<CvMatMessage::ConstPtr>);

    if(!loaded_) {
        throw std::runtime_error("No SVM is loaded!");
    }



    const bool compute_label = readParameter<bool>("compute labels");
    std::function<bool(float, float)> comparator;

    {
        const int comparison_type = readParameter<int>("threshold/type");

        switch (comparison_type)
        {
        default:
        case GREATER:
            comparator = [](float value, float threshold) { return value > threshold; };
            break;
        case GREATER_EQUAL:
            comparator = [](float value, float threshold) { return value >= threshold; };
            break;
        case LESS:
            comparator = [](float value, float threshold) { return value < threshold; };
            break;
        case LESS_EQUAL:
            comparator = [](float value, float threshold) { return value <= threshold; };
            break;
        }
    }

    /// TODO : OMP?

    std::size_t size = input->size();
    for(std::size_t i = 0 ; i < size ; ++i)
    {
        cv::Mat sample(input->at(i).value);
        CvMatMessage::Ptr result_msg(new CvMatMessage(enc::unknown, 0));
        cv::Mat &result_value = result_msg->value;
        result_value = svm_responses_.clone();

        for(std::size_t j = 0 ; j < svms_size_ ; ++j) {
            double           thresh = thresholds_[j];

#if CV_MAJOR_VERSION == 2
            ExtendedSVM::Ptr svm = svms_.at(j);
#elif CV_MAJOR_VERSION == 3
            SVMPtr svm = svms_.at(j);
#endif
            if (compute_label) {
                result_value.at<float>(j,1) = svm->predict(sample);
            } else {
#if CV_MAJOR_VERSION == 2
                const float response = svm->predict(sample, true);
#else
                const float response = svm->predict(sample, cv::noArray(), cv::ml::StatModel::RAW_OUTPUT);
#endif
                result_value.at<float>(j,1) = comparator(response, thresh) ? POSITIVE : NEGATIVE;
            }
        }
        output->push_back(result_msg);
    }

    msg::publish<GenericVectorMessage, CvMatMessage::ConstPtr>(out_, output);
}

void SVMEnsemble::load()
{
    std::string path = readParameter<std::string>("svm array path");
    if(path == "")
        return;

    svms_.clear();
    const static std::string prefix = "svm_";
    cv::FileStorage fs(path, cv::FileStorage::READ);
    std::vector<int> labels;
    fs["labels"] >> labels;

    if(labels.empty())
        throw std::runtime_error("File entry 'labels' may not be empty!");

    /// get svms for labels
    svm_responses_ = cv::Mat(labels.size(), 2, CV_32FC1, cv::Scalar());

    //// find the minimum floating point precision
    double min_rho = std::numeric_limits<double>::max();
    for(std::size_t i = 0 ; i < labels.size() ; ++i) {
        std::string label = prefix + std::to_string(labels.at(i));

#if CV_MAJOR_VERSION == 2
        ExtendedSVM::Ptr svm(new ExtendedSVM);
        svm->read(fs.fs, (CvFileNode*) fs[label].node);
#elif CV_MAJOR_VERSION == 3
        SVMPtr svm = cv::ml::SVM::create();
        svm->read(*(cv::FileNode*) fs[label].node);
#endif

        svms_.push_back(svm);
        svm_responses_.at<float>(i, 0) = labels.at(i);

#if CV_MAJOR_VERSION == 2
        auto rho = svm->rho();
#elif CV_MAJOR_VERSION == 3
        auto rho = svm->getDecisionFunction(0, cv::noArray(), cv::noArray());
#endif

        if(rho < min_rho)
            min_rho = fabs(rho);
    }

    if(!params_thresholds_.empty()) {
        removeTemporaryParameters();
        params_thresholds_.clear();
    }

    double step = std::pow(10,ceil(log10(min_rho)))*1e-7;/*.1;*/
    std::cout << min_rho << std::endl;
    std::cout << step << std::endl;
//    std::size_t digits = impl::digits(min_rho);
//    std::cout << digits << std::endl;
//    for(std::size_t i = 0 ; i < digits ; ++i) {
//        step *= .1;
//    }

    svms_size_ = svms_.size();
    thresholds_.resize(svms_size_);
    for(std::size_t i = 0 ; i < svms_size_ ; ++i) {
        std::string id = std::to_string((int) svm_responses_.at<float>(i,0));
        param::Parameter::Ptr param = param::ParameterFactory::declareRange("svm_" + id,
                                                                            -100.0,
                                                                            +100.0,
                                                                            0.0,
                                                                            step);
        param::RangeParameter::Ptr thresh =
                std::dynamic_pointer_cast<param::RangeParameter>(param);

        if(!thresh) {
            throw std::runtime_error("Could not create temporary parameter!");
        }

        params_thresholds_.emplace_back(thresh);
        addTemporaryParameter(param, std::bind(&SVMEnsemble::updateThresholds, this));
    }


    loaded_ = true;

}

void SVMEnsemble::updateThresholds()
{
    for(std::size_t i = 0 ; i < svms_size_ ; ++i) {
        thresholds_[i] = params_thresholds_[i]->as<double>();
    }
}
