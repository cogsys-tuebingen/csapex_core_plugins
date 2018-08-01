/// HEADER
#include "random_trees_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <opencv2/ml/ml.hpp>

CSAPEX_REGISTER_CLASS(csapex::RandomTreesTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


RandomTreesTrainer::RandomTreesTrainer()
    : MachineLearningNode("rforest.yaml"), categories_(0)
{
}


void RandomTreesTrainer::setupParameters(Parameterizable& parameters)
{
    MachineLearningNode::setupParameters(parameters);

    addParameter(csapex::param::factory::declareRange<int>
                 ("classes",
                  csapex::param::ParameterDescription("Number of classes to learn."),
                  0, 100, 2, 1),
                 std::bind(&RandomTreesTrainer::updatePriors, this));;

    addParameter(csapex::param::factory::declareRange<int>
                 ("max depth",
                  csapex::param::ParameterDescription("the depth of the tree. A low value will likely underfit and conversely a high value will likely overfit.\n"
                                                      "The optimal value can be obtained using cross validation or other suitable methods."),
                  1, 64, 8, 1));;
    addParameter(csapex::param::factory::declareRange<int>
                 ("min sample count",
                  csapex::param::ParameterDescription("minimum samples required at a leaf node for it to be split.\n"
                                                      "A reasonable value is a small percentage of the total data e.g. 1%."),
                  0, 64, 10, 1));
    addParameter(csapex::param::factory::declareRange<double>
                 ("regression accuracy",
                  csapex::param::ParameterDescription("Termination criteria for regression trees. \n"
                                                      "If all absolute differences between an estimated value in a node and values of train samples in this node \n"
                                                      "are less than this parameter then the node will not be split."),
                  0.0, 255.0, 0.0, 0.01));;

#if CV_MAJOR_VERSION == 2
    bool use_surrogates = true;
#elif CV_MAJOR_VERSION == 3
    bool use_surrogates = false; // not yet supported
#endif

    auto surrogate_p = csapex::param::factory::declareBool
            ("use surrogates",
             csapex::param::ParameterDescription("If true then surrogate splits will be built. \n"
                                                 "These splits allow to work with missing data and compute variable importance correctly."),
             use_surrogates).build();
    addParameter(surrogate_p, use_surrogates);

#if CV_MAJOR_VERSION == 3
    surrogate_p->setHidden(true);
#endif



    addParameter(csapex::param::factory::declareRange<int>
                 ("max categories",
                  csapex::param::ParameterDescription("Cluster possible values of a categorical variable into K <= max_categories clusters to find a suboptimal split.\n"
                                                      "If a discrete variable, on which the training procedure tries to make a split, takes more than max_categories values,\n"
                                                      "the precise best subset estimation may take a very long time because the algorithm is exponential.\n"
                                                      "Instead, many decision trees engines (including ML) try to find sub-optimal split in this case by clustering all\n"
                                                      "the samples into max_categories clusters that is some categories are merged together.\n"
                                                      "The clustering is applied only in n>2-class classification problems for categorical variables\n"
                                                      "with N > max_categories possible values.\n"
                                                      "In case of regression and 2-class classification the optimal split can be found efficiently without employing clustering,\n"
                                                      "thus the parameter is not used in these cases."),
                  0, 100, 15, 1));;

    addParameter(csapex::param::factory::declareBool
                 ("calc_var_importance",
                  csapex::param::ParameterDescription("If true then variable importance will be calculated and then it can be retrieved by CvRTrees::get_var_importance()."),
                  false));

    addParameter(csapex::param::factory::declareRange<int>
                 ("nactive_vars",
                  csapex::param::ParameterDescription("The size of the randomly selected subset of features at each tree node and that are used to find the best split(s).\n"
                                                      "If you set it to 0 then the size will be set to the square root of the total number of features."),
                  0, 100, 0, 1));
    addParameter(csapex::param::factory::declareRange<int>
                 ("max_num_of_trees_in_the_forest",
                  csapex::param::ParameterDescription("The maximum number of trees in the forest (surprise, surprise). Typically the more trees you have the better the accuracy.\n"
                                                      "However, the improvement in accuracy generally diminishes and asymptotes pass a certain number of trees.\n"
                                                      "Also to keep in mind, the number of tree increases the prediction time linearly."),
                  1, 1024, 16, 1));
    addParameter(csapex::param::factory::declareRange<double>
                 ("forest_accuracy",
                  csapex::param::ParameterDescription("Sufficient accuracy (OOB error)."),
                  0.0, 1.0, 0.5, 0.01));


    std::map<std::string, int> termcrit_type = {
        {"CV_TERMCRIT_ITER", (int) CV_TERMCRIT_ITER},
        {"CV_TERMCRIT_EPS", (int) CV_TERMCRIT_EPS},
        {"CV_TERMCRIT_ITER | CV_TERMCRIT_EPS", (int) CV_TERMCRIT_ITER | CV_TERMCRIT_EPS}
    };


    csapex::param::Parameter::Ptr termcrit_type_p = csapex::param::factory::declareParameterSet
            ("termcrit_type",
             csapex::param::ParameterDescription("The type of the termination criteria:\n"
                                                 "CV_TERMCRIT_ITER Terminate learning by the max_num_of_trees_in_the_forest;\n"
                                                 "CV_TERMCRIT_EPS Terminate learning by the forest_accuracy;\n"
                                                 "CV_TERMCRIT_ITER | CV_TERMCRIT_EPS Use both termination criteria."),
             termcrit_type, (int) (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS));
    parameters.addParameter(termcrit_type_p);
}

void RandomTreesTrainer::updatePriors()
{
    int categories = readParameter<int>("classes");

    if(categories != categories_) {
        if(categories > categories_) {
            for(int c = categories_; c < categories; ++c) {
                std::stringstream name;
                name << "~priors/" << c;
                csapex::param::Parameter::Ptr p = csapex::param::factory::declareRange<double>(name.str(), 0.0, 50.0, 1.0, 0.01);
                priors_params_.push_back(p);
                addTemporaryParameter(p, std::bind(&RandomTreesTrainer::udpatePriorValues, this));
            }
        } else {
            for(int c = categories_-1; c >= categories; --c) {
                removeTemporaryParameter(priors_params_[c]);
                priors_params_.pop_back();
            }
        }
        categories_ = categories;
    }
}

void RandomTreesTrainer::udpatePriorValues()
{
    if(priors_.size() != priors_params_.size()) {
        priors_.resize(priors_params_.size());
    }

    for(std::size_t i = 0, total = priors_.size(); i < total; ++i) {
        priors_[i] = priors_params_[i]->as<double>();
    }
}

bool RandomTreesTrainer::processCollection(std::vector<connection_types::FeaturesMessage>& collection)
{
    FeaturesMessage& first_feature = collection[0];
    std::size_t feature_length = first_feature.value.size();
    std::size_t responses_length = first_feature.regression_result.size();
    std::size_t collection_size = collection.size();
    std::size_t var_type_size = 0;

    cv::Mat train_data(collection_size, feature_length, CV_32FC1, cv::Scalar());
    cv::Mat missing(collection_size, feature_length, CV_8UC1, cv::Scalar(0));
    cv::Mat responses;
    if(is_classification_){
        // classification problem
        var_type_size = 1;
        responses = cv::Mat(collection_size, var_type_size, CV_32SC1, cv::Scalar());
    }
    else{
        // regression problem
        var_type_size = responses_length;
        responses = cv::Mat(collection_size, var_type_size, CV_32FC1, cv::Scalar());
    }

#if CV_MAJOR_VERSION == 2
    int tflag = CV_ROW_SAMPLE;
#elif CV_MAJOR_VERSION == 3
    int tflag = cv::ml::ROW_SAMPLE;
#endif

    std::set<int> classifications;
    for(std::size_t i = 0; i < collection_size; ++i) {
        FeaturesMessage& feature = collection[i];
        for(std::size_t j = 0; j < feature_length; ++j) {
            const float& val = feature.value[j];

            if(std::abs(val) >= FLT_MAX*0.5f) {
                missing.at<uchar>(i,j) = 1;
            } else {
                train_data.at<float>(i,j) = val;
            }
        }
        if(is_classification_){ // classification problem
            responses.at<int>(i,0) = feature.classification;
            classifications.insert(feature.classification);
        }
        else{ // regression problem
            for(std::size_t j = 0; j < responses_length; ++j){
                const float& val = feature.regression_result[j];

                if(std::abs(val) < FLT_MAX*0.5f) {
                    responses.at<float>(i,j) = val;
                }
            }
        }
    }

#if CV_MAJOR_VERSION == 2
    CvRTParams params( readParameter<int>("max depth"),
                       readParameter<int>("min sample count"),
                       readParameter<double>("regression accuracy"),
                       readParameter<bool>("use surrogates"),
                       readParameter<int>("max categories"),
                       priors_.data(),
                       readParameter<bool>("calc_var_importance"),
                       readParameter<int>("nactive_vars"),
                       readParameter<int>("max_num_of_trees_in_the_forest"),
                       readParameter<double>("forest_accuracy"),
                       readParameter<int>("termcrit_type"));

    cv::Mat var_type( train_data.cols + 1, 1, CV_8U, CV_VAR_NUMERICAL);

    cv::RandomTrees rtrees;
    ainfo << "Started training of tree with feature width "
         << collection.front().value.size() << " and " << collection_size << " samples!" << std::endl;
    if(rtrees.train(train_data, tflag, responses, cv::Mat(), cv::Mat(), var_type, cv::Mat(), params)) {
        cv::FileStorage fs(file_name_, cv::FileStorage::WRITE);
        rtrees.write(fs.fs, "random_forest");
        fs << "classes" << "[";
        for(int i : classifications) {
            fs << i;
        }
        fs << "]";
        fs.release();
        std::cout << "[RandomTrees]: Finished training!" << std::endl;
    } else {
        return false;
    }

#elif CV_MAJOR_VERSION == 3
    auto rtrees = cv::ml::RTrees::create();
    rtrees->setMaxDepth(readParameter<int>("max depth"));
    rtrees->setMinSampleCount(readParameter<int>("min sample count"));
    rtrees->setRegressionAccuracy(readParameter<double>("regression accuracy"));
    rtrees->setUseSurrogates(false); // not calling readParameter<bool>("use surrogates") -> not implemented
    rtrees->setMaxCategories(readParameter<int>("max categories"));
    rtrees->setCalculateVarImportance(readParameter<bool>("calc_var_importance"));
    rtrees->setActiveVarCount(readParameter<int>("nactive_vars"));

    cv::TermCriteria term(readParameter<int>("termcrit_type"), readParameter<int>("max_num_of_trees_in_the_forest"), readParameter<double>("forest_accuracy"));
    rtrees->setTermCriteria(term);

    cv::Mat priors(priors_);
    if(is_classification_){
        rtrees->setPriors(priors);
    }
    else{
        rtrees->setPriors(cv::Mat());
    }

    cv::Mat var_type;
    var_type = cv::Mat( train_data.cols + var_type_size, 1, CV_8U, cv::ml::VAR_NUMERICAL);

    cv::Ptr<cv::ml::TrainData> train_data_struct = cv::ml::TrainData::create(train_data,
                                                                             tflag,
                                                                             responses,
                                                                             cv::noArray(), cv::noArray(), cv::noArray(),
                                                                             var_type);

    ainfo << "Started training with " << train_data.rows << " samples!" << std::endl;
    if(rtrees->train(train_data_struct)) {
        cv::FileStorage fs(file_name_, cv::FileStorage::WRITE);
        rtrees->write(fs);
        ainfo << "Finished training!" << std::endl;
        ainfo << "Random forest was written to " << file_name_ << std::endl;
    } else {
        return false;
    }
#endif

    return true;
}

