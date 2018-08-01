#include "mlp_cv.hpp"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/slot.h>

CSAPEX_REGISTER_CLASS(csapex::MLPCv, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MLPCv::MLPCv()
    : loaded_(false)
{
}

void MLPCv::setup(NodeModifier &node_modifier)
{
    input_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("Unclassified feature");
    output_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Classified features");

    reload_ = node_modifier.addSlot("Reload", std::bind(&MLPCv::reloadMLP, this));
}

void MLPCv::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::factory::declareFileInputPath("file", "mlp.yaml"),
                            [this](param::Parameter* p) {
        auto path = p->as<std::string>();
        if(path != path_){
            path_ = path;
            reloadMLP();
        }
    });
}

void MLPCv::loadMLP()
{
#if CV_MAJOR_VERSION == 2
    mlp_.load(path_.c_str());
    loaded_ = mlp_.get_layer_count() > 0; // oddly opencv does not check if file is valid
#elif CV_MAJOR_VERSION == 3
    mlp_ = cv::ml::ANN_MLP::load(path_);
    cv::Mat sizes = mlp_->getLayerSizes();
    loaded_ = sizes.rows > 0 || sizes.cols > 0;

#endif

}

void MLPCv::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input =
            msg::getMessage<GenericVectorMessage, FeaturesMessage>(input_);

    std::shared_ptr<std::vector<FeaturesMessage>> output(new std::vector<FeaturesMessage>);
    if(!loaded_) {
        if(path_ != "") {
            loadMLP();
        }
    }

    if(loaded_) {

        std::size_t n = input->size();
        output->resize(n);
        for(std::size_t i = 0; i < n; ++i) {
            classify(input->at(i),output->at(i));
        }

    } else {
        *output = *input;
        node_modifier_->setWarning("cannot classfiy, no MLP loaded");
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(output_, output);
}

void MLPCv::classify(const FeaturesMessage &input,
                     FeaturesMessage &output)
{
    output = input;

    cv::Mat feature(1, input.value.size(),cv::DataType<float>::type);
    for(std::size_t i = 0; i < input.value.size(); ++i){
        feature.at<float>(0,i) = input.value[i];
    }
    cv::Mat response;


#if CV_MAJOR_VERSION == 2
    mlp_.predict(feature, response);
#elif CV_MAJOR_VERSION == 3
    mlp_->predict(feature, response);
#endif
    if(input.type == FeaturesMessage::Type::REGRESSION) {
        std::size_t rows = response.rows;
        std::size_t cols = response.cols;
        output.regression_result.clear();
        output.regression_result.resize(rows * cols);
        for(std::size_t i = 0; i < rows; ++ i) {
            for(std::size_t j = 0; j < cols; ++j) {
                output.regression_result.at(i*cols + j) = response.at<float>(i,j);
            }
        }
    } else {
        cv::Point max;
        cv::minMaxLoc(response, nullptr, nullptr, nullptr, &max);

        output.classification = max.x;
    }
}

void MLPCv::reloadMLP()
{
    loaded_ = false;
}
