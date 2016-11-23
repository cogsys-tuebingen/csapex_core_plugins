#ifndef SVM_TRAINER_H
#define SVM_TRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex_core_plugins/collection_node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN SVMTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    SVMTrainer();

    void setup(NodeModifier &modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    std::string     path_;
    bool            save_for_hog_;

    int         svm_type;
    int         kernel_type;
    double      degree; // for poly
    double      gamma;  // for poly/rbf/sigmoid
    double      coef0;  // for poly/sigmoid

    double      C;  // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
    double      nu; // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
    double      p; // for CV_SVM_EPS_SVR
//    CvMat*      class_weights; // for CV_SVM_C_SVC
    CvTermCriteria term_crit; // termination criteria

    bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;

};
}

#endif // SVM_TRAINER_H
