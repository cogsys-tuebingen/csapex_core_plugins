#ifndef SVM_ARRAY_TRAINER_H
#define SVM_ARRAY_TRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include "random_vector.hpp"
#include <csapex_core_plugins/collection_node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN SVMEnsembleTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    SVMEnsembleTrainer();

    void setup(NodeModifier& modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    std::string path_;
    bool one_vs_all_;
    bool balance_;

    int svm_type_;
    int kernel_type_;
    double degree_;  // for poly
    double gamma_;   // for poly/rbf/sigmoid
    double coef0_;   // for poly/sigmoid

    double C_;            // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
    double nu_;           // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
    double p_;            // for CV_SVM_EPS_SVR
                          //    CvMat*      class_weights; // for CV_SVM_C_SVC
    double epsilon_;      // termination criteria accuracy
    int term_crit_type_;  // termination criteria type
    int term_crit_iterations_;
    cv::TermCriteria term_crit_;  // termination criteria

    enum ClassTypes
    {
        NEGATIVE = -1,
        POSITIVE = 1
    };

    bool processCollection(std::vector<connection_types::FeaturesMessage>& collection) override;
    RandomVector rand_vec_;
};
}  // namespace csapex

#endif  // SVM_ARRAY_TRAINER_H
