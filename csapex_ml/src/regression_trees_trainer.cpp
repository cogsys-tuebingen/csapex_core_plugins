
/// HEADER
#include "regression_trees_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <opencv2/ml/ml.hpp>
#include <map>

CSAPEX_REGISTER_CLASS(csapex::RegressionTreesTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


RegressionTreesTrainer::RegressionTreesTrainer()
{
}


void RegressionTreesTrainer::setupParameters(csapex::Parameterizable& params)
{

//    params.addParameter(param::);
    MachineLearningNode::setupParameters(params);
    params.addParameter(csapex::param::ParameterFactory::declareRange<int>
                        ("max depth",
                         csapex::param::ParameterDescription("the depth of the tree. A low value will likely underfit and conversely a high value will likely overfit.\n"
                                                             "The optimal value can be obtained using cross validation or other suitable methods."),
                         1, 64, 8, 1));;
    params.addParameter(csapex::param::ParameterFactory::declareRange<int>
                        ("min sample count",
                         csapex::param::ParameterDescription("minimum samples required at a leaf node for it to be split.\n"
                                                             "A reasonable value is a small percentage of the total data e.g. 1%."),
                         0, 64, 10, 1));
    params.addParameter(csapex::param::ParameterFactory::declareRange<double>
                        ("regression accuracy",
                         csapex::param::ParameterDescription("Termination criteria for regression trees. \n"
                                                             "If all absolute differences between an estimated value in a node and values of train samples in this node \n"
                                                             "are less than this parameter then the node will not be split."),
                         0.0, 255.0, 0.0, 0.01));;

#if CV_MAJOR_VERSION == 2
#elif CV_MAJOR_VERSION == 3
    bool use_surrogates = false; // not yet supported
#endif

    params.addParameter(csapex::param::ParameterFactory::declareBool
                        ("use surrogates",
                         csapex::param::ParameterDescription("If true then surrogate splits will be built. \n"
                                                             "These splits allow to work with missing data and compute variable importance correctly."),
                         use_surrogates));
    params.addParameter(csapex::param::ParameterFactory::declareRange<int>
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

    params.addParameter(csapex::param::ParameterFactory::declareBool
                        ("calc_var_importance",
                         csapex::param::ParameterDescription("If true then variable importance will be calculated and then it can be retrieved by CvRTrees::get_var_importance()."),
                         false));

    params.addParameter(csapex::param::ParameterFactory::declareRange<int>
                        ("nactive_vars",
                         csapex::param::ParameterDescription("The size of the randomly selected subset of features at each tree node and that are used to find the best split(s).\n"
                                                             "If you set it to 0 then the size will be set to the square root of the total number of features."),
                         0, 100, 0, 1));
    params.addParameter(csapex::param::ParameterFactory::declareRange<int>
                        ("max_num_of_trees_in_the_forest",
                         csapex::param::ParameterDescription("The maximum number of trees in the forest (surprise, surprise). Typically the more trees you have the better the accuracy.\n"
                                                             "However, the improvement in accuracy generally diminishes and asymptotes pass a certain number of trees.\n"
                                                             "Also to keep in mind, the number of tree increases the prediction time linearly."),
                         1, 1024, 16, 1));
    params.addParameter(csapex::param::ParameterFactory::declareRange<double>
                        ("forest_accuracy",
                         csapex::param::ParameterDescription("Sufficient accuracy (OOB error)."),
                         0.0, 1.0, 0.5, 0.01));


    std::map<std::string, int> termcrit_type = {
        {"CV_TERMCRIT_ITER", (int) CV_TERMCRIT_ITER},
        {"CV_TERMCRIT_EPS", (int) CV_TERMCRIT_EPS},
        {"CV_TERMCRIT_ITER | CV_TERMCRIT_EPS", (int) CV_TERMCRIT_ITER | CV_TERMCRIT_EPS}
    };


    csapex::param::Parameter::Ptr termcrit_type_p = csapex::param::ParameterFactory::declareParameterSet
            ("termcrit_type",
             csapex::param::ParameterDescription("The type of the termination criteria:\n"
                                                 "CV_TERMCRIT_ITER Terminate learning by the max_num_of_trees_in_the_forest;\n"
                                                 "CV_TERMCRIT_EPS Terminate learning by the forest_accuracy;\n"
                                                 "CV_TERMCRIT_ITER | CV_TERMCRIT_EPS Use both termination criteria."),
             termcrit_type, (int) (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS));
    params.addParameter(termcrit_type_p);
}



bool RegressionTreesTrainer::processCollection(std::vector<connection_types::FeaturesMessage>& collection)
{
    FeaturesMessage& first_feature = collection[0];

    apex_assert(first_feature.type == FeaturesMessage::Type::REGRESSION);
    std::size_t feature_length = first_feature.value.size();
    std::size_t responses_length = first_feature.regression_result.size();
    std::size_t collection_size = collection.size();
    std::size_t var_type_size = 1;

    cv::Mat train_data(collection_size, feature_length, CV_32FC1, cv::Scalar());
    cv::Mat missing(collection_size, feature_length, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Mat> responses(responses_length);
    for(auto it = responses.begin(); it < responses.end(); ++it){
        *it = cv::Mat(collection_size, var_type_size, CV_32FC1, cv::Scalar());
    }

    cv::FileStorage fs(file_name_, cv::FileStorage::WRITE);
    const static std::string prefix = "reg_forest_";
    fs << "regression" << "{";

#if CV_MAJOR_VERSION == 2
#elif CV_MAJOR_VERSION == 3
    int tflag = cv::ml::ROW_SAMPLE;
#endif

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
        for(std::size_t j = 0; j < responses_length; ++j){
            const float& val = feature.regression_result[j];

            if(std::abs(val) < FLT_MAX*0.5f) {
                responses.at(j).at<float>(i,0) = val;
            }
        }
    }
    for(std::size_t i = 0; i < responses_length; ++i){

#if CV_MAJOR_VERSION == 2
        #pragma message "Regression Trees are currently not implemented for OpenCV 2."
#elif CV_MAJOR_VERSION == 3
        cv::Ptr<cv::ml::RTrees> rtrees = cv::ml::RTrees::create();
        rtrees->setMaxDepth(readParameter<int>("max depth"));
        rtrees->setMinSampleCount(readParameter<int>("min sample count"));
        rtrees->setRegressionAccuracy(readParameter<double>("regression accuracy"));
        rtrees->setUseSurrogates(readParameter<bool>("use surrogates"));
        rtrees->setMaxCategories(readParameter<int>("max categories"));
        rtrees->setCalculateVarImportance(readParameter<bool>("calc_var_importance"));
        rtrees->setActiveVarCount(readParameter<int>("nactive_vars"));

        cv::TermCriteria term(readParameter<int>("termcrit_type"), readParameter<int>("max_num_of_trees_in_the_forest"), readParameter<double>("forest_accuracy"));
        rtrees->setTermCriteria(term);

        //    cv::Mat priors(priors_);
        //    if(is_classification_){
        //        rtrees->setPriors(priors);
        //    }

        cv::Mat var_type = cv::Mat( train_data.cols + var_type_size, 1, CV_8U, cv::ml::VAR_NUMERICAL);

        std::cout << std::endl;
        cv::Ptr<cv::ml::TrainData> train_data_struct = cv::ml::TrainData::create(train_data,
                                                                                 tflag,
                                                                                 responses.at(i),
                                                                                 cv::noArray(), cv::noArray(), cv::noArray(),
                                                                                 var_type);

        ainfo << "Started training for tree # "<< i
              << " with " << train_data.rows << " samples!" << std::endl;
        if(rtrees->train(train_data_struct)) {
            std::string label = prefix + std::to_string(i);
            fs << label << "{";
            rtrees->write(fs);
            fs << "}";
            ainfo << "Finished training for tree # "
                  << i <<"!" << std::endl;
        } else {
            return false;
        }
#endif
    }

    fs << "}";
    fs << "num_forests" << (int) responses_length;
    fs.release();
    return true;
}


