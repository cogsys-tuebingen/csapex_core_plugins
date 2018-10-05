/// HEADER
#include "mlp_cv_trainer.hpp"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <opencv2/ml/ml.hpp>

CSAPEX_REGISTER_CLASS(csapex::MLPCvTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MLPCvTrainer::MLPCvTrainer() : layers_(0)
{
}

void MLPCvTrainer::setup(NodeModifier& node_modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(node_modifier);
}

void MLPCvTrainer::setupParameters(Parameterizable& parameters)
{
    CollectionNode<connection_types::FeaturesMessage>::setupParameters(parameters);

    parameters.addParameter(csapex::param::factory::declareFileOutputPath("file", "mlp.yaml"));

    parameters.addParameter(csapex::param::factory::declareRange<int>("classes", csapex::param::ParameterDescription("Number of classes to learn."), 0, 100, 2, 1));

    parameters.addParameter(csapex::param::factory::declareRange<int>("layers", csapex::param::ParameterDescription("Number of hidden layers"), 0, 100, 1, 1),
                            std::bind(&MLPCvTrainer::updateLayers, this));

#if CV_MAJOR_VERSION == 2
    typedef cv::ANN_MLP_TrainParams Params;
    typedef cv::NeuralNet_MLP Ann;
#elif CV_MAJOR_VERSION == 3
    typedef cv::ml::ANN_MLP Params;
    typedef cv::ml::ANN_MLP Ann;
#endif

    std::map<std::string, int> training_method = {
        { "backprop", Params::BACKPROP },
        { "rprop", Params::RPROP },
    };

    csapex::param::Parameter::Ptr training_parameter = csapex::param::factory::declareParameterSet<int>("trainig method", training_method, Params::BACKPROP);
    parameters.addParameter(training_parameter);

    parameters.addParameter(csapex::param::factory::declareRange<double>("training param 1", 0, 10, 0.1, 0.01));
    parameters.addParameter(csapex::param::factory::declareRange<double>("training param 2", 0, 10, 0.1, 0.01));

    std::map<std::string, int> activation_functions = {
        { "identity", Ann::IDENTITY },
        { "sigmoid", Ann::SIGMOID_SYM },
        { "gaussian", Ann::GAUSSIAN },
    };

    csapex::param::Parameter::Ptr activation_parameter = csapex::param::factory::declareParameterSet<int>("activation function", activation_functions, Ann::SIGMOID_SYM);
    parameters.addParameter(activation_parameter);

    parameters.addConditionalParameter(csapex::param::factory::declareRange<double>("activation alpha", -10, 10, 1.0, 0.01), [=]() { return activation_parameter->as<int>() != Ann::IDENTITY; });
    parameters.addConditionalParameter(csapex::param::factory::declareRange<double>("activation beta", -10, 10, 1.0, 0.01), [=]() { return activation_parameter->as<int>() != Ann::IDENTITY; });

    std::map<std::string, int> termcrit_type = { { "CV_TERMCRIT_ITER", (int)CV_TERMCRIT_ITER },
                                                 { "CV_TERMCRIT_EPS", (int)CV_TERMCRIT_EPS },
                                                 { "CV_TERMCRIT_ITER | CV_TERMCRIT_EPS", (int)CV_TERMCRIT_ITER | CV_TERMCRIT_EPS } };

    csapex::param::Parameter::Ptr termcrit_parameter = csapex::param::factory::declareParameterSet("termcrit_type",
                                                                                                   csapex::param::ParameterDescription("The type of the termination criteria:\n"
                                                                                                                                       "CV_TERMCRIT_ITER Terminate learning by the "
                                                                                                                                       "max_num_of_trees_in_the_forest;\n"
                                                                                                                                       "CV_TERMCRIT_EPS Terminate learning by the forest_accuracy;\n"
                                                                                                                                       "CV_TERMCRIT_ITER | CV_TERMCRIT_EPS Use both termination "
                                                                                                                                       "criteria."),
                                                                                                   termcrit_type, (int)(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS));
    parameters.addParameter(termcrit_parameter);

    parameters.addConditionalParameter(
        csapex::param::factory::declareRange<int>("max iterations", csapex::param::ParameterDescription("Maximum number of training iterations."), 0, 1000000, 10000, 100),
        [=]() { return (termcrit_parameter->as<int>() & CV_TERMCRIT_ITER) != 0; });

    parameters.addConditionalParameter(csapex::param::factory::declareValue<double>("epsilon", csapex::param::ParameterDescription("Training epsilon"), 0.001),
                                       [=]() { return (termcrit_parameter->as<int>() & CV_TERMCRIT_EPS) != 0; });
}

void MLPCvTrainer::updateLayers()
{
    int layers = readParameter<int>("layers");

    if (layers != layers_) {
        if (layers > layers_) {
            for (int l = layers_; l < layers; ++l) {
                std::stringstream name;
                name << "~layer/" << l;
                csapex::param::Parameter::Ptr p = csapex::param::factory::declareRange<int>(name.str(), 0, 10000, 10, 1);
                layer_params_.push_back(p);
                addTemporaryParameter(p);
            }
        } else {
            for (int l = layers_ - 1; l >= layers; --l) {
                removeTemporaryParameter(layer_params_[l]);
                layer_params_.pop_back();
            }
        }

        layers_ = layers;
    }
}

bool MLPCvTrainer::processCollection(std::vector<FeaturesMessage>& collection)
{
    const int classes = readParameter<int>("classes");

    FeaturesMessage& first_feature = collection[0];
    std::size_t feature_length = first_feature.value.size();

    cv::Mat trainig_data(collection.size(), feature_length, CV_32FC1);

    std::size_t responses_cols = 0;
    if (first_feature.type == FeaturesMessage::Type::CLASSIFICATION) {
        responses_cols = classes;
    } else if (first_feature.type == FeaturesMessage::Type::REGRESSION) {
        responses_cols = first_feature.regression_result.size();
    }
    cv::Mat responses(collection.size(), responses_cols, CV_32FC1);

    for (std::size_t i = 0; i < collection.size(); ++i) {
        const FeaturesMessage& feature = collection[i];
        for (std::size_t j = 0; j < feature_length; ++j) {
            const float& value = feature.value[j];

            trainig_data.at<float>(i, j) = value;
        }
        if (first_feature.type == FeaturesMessage::Type::CLASSIFICATION) {
            for (int j = 0; j < classes; ++j) {
                if (j == feature.classification)
                    responses.at<float>(i, j) = 1;
                else
                    responses.at<float>(i, j) = 0;
            }
        } else if (first_feature.type == FeaturesMessage::Type::REGRESSION) {
            apex_assert_eq(feature.regression_result.size(), responses_cols);
            auto it = feature.regression_result.begin();
            for (std::size_t j = 0; j < responses_cols; ++j, ++it) {
                responses.at<float>(i, j) = *it;
            }
        }
    }

#if CV_MAJOR_VERSION == 2
    cv::ANN_MLP_TrainParams params(cv::TermCriteria(readParameter<int>("termcrit_type"), readParameter<int>("max iterations"), readParameter<double>("epsilon")), readParameter<int>("trainig method"),
                                   readParameter<double>("training param 1"), readParameter<double>("training param 2"));

    cv::Mat layers(1, 1 + layers_ + 1, CV_32SC1);
    layers.at<int>(0, 0) = feature_length;
    for (int l = 0; l < layers_; ++l)
        layers.at<int>(0, 1 + l) = layer_params_[l]->as<int>();
    layers.at<int>(0, layers.cols - 1) = classes;

    cv::NeuralNet_MLP mlp;
    mlp.create(layers, readParameter<int>("activation function"), readParameter<double>("activation alpha"), readParameter<double>("activation beta"));

    std::cout << "[ANN]: Started training with " << trainig_data.rows << " samples!" << std::endl;
    int iters = mlp.train(trainig_data, responses, cv::Mat(), cv::Mat(),
                          params);  //,
    // 0);
    ainfo << "iterations: " << iters << std::endl;
    if (iters > 0) {
        mlp.save(readParameter<std::string>("file").c_str());
        std::cout << "[MLP_ANN]: Finished training after " << iters << " iterations!" << std::endl;
    } else {
        return false;
    }
    return true;

#elif CV_MAJOR_VERSION == 3
    cv::Ptr<cv::ml::ANN_MLP> mlp = cv::ml::ANN_MLP::create();
    cv::TermCriteria term(readParameter<int>("termcrit_type"), readParameter<int>("max iterations"), readParameter<double>("epsilon"));
    mlp->setTermCriteria(term);

    cv::Mat layers(1, 1 + layers_ + 1, CV_32SC1);
    layers.at<int>(0, 0) = feature_length;
    for (int l = 0; l < layers_; ++l)
        layers.at<int>(0, 1 + l) = layer_params_[l]->as<int>();
    layers.at<int>(0, layers.cols - 1) = classes;

    mlp->setActivationFunction(readParameter<int>("activation function"), readParameter<double>("activation alpha"), readParameter<double>("activation beta"));

    mlp->setLayerSizes(layers);
    mlp->setTrainMethod(readParameter<int>("trainig method"), readParameter<double>("training param 1"), readParameter<double>("training param 2"));

    std::cout << "[ANN]: Started training with " << trainig_data.rows << " samples!" << std::endl;

    cv::Ptr<cv::ml::TrainData> train_data_struct = cv::ml::TrainData::create(trainig_data, cv::ml::ROW_SAMPLE, responses, cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray());

    if (mlp->train(train_data_struct)) {
        mlp->save(readParameter<std::string>("file"));
        std::cout << "[MLP_ANN]: Finished training!" << std::endl;
    } else {
        return false;
    }
    return true;
#endif

    /*
    auto print_res = [](const cv::Mat& mat)
    {
        std::stringstream ss;
        for (int i = 0; i < mat.cols; ++i)
            ss << mat.at<float>(0, i) << ", ";
        return ss.str();
    };

    for (int i = 0; i < trainig_data.rows; ++i)
    {
        cv::Mat input;
        cv::Mat output;

        input = trainig_data.row(i);

        mlp.predict(input, output);

        ainfo << "Output: " << print_res(output) << std::endl;
        ainfo << "Should: " << print_res(responses.row(i)) << std::endl;

        cv::Point output_class;
        cv::minMaxLoc(output, nullptr, nullptr, nullptr, &output_class);

        cv::Point should_class;
        cv::minMaxLoc(responses.row(i), nullptr, nullptr, nullptr, &should_class);
    }
  */
}
