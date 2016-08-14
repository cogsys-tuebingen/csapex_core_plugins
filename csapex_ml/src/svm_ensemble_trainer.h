#ifndef SVM_ARRAY_TRAINER_H
#define SVM_ARRAY_TRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex_core_plugins/collection_node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class SVMEnsembleTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    SVMEnsembleTrainer();

    void setupParameters(Parameterizable& parameters);

private:
    std::string     path_;
    bool            save_for_hog_;
    bool            one_vs_all_;
    cv::SVMParams   svm_params_;


    enum ClassTypes {NEGATIVE = -1, POSITIVE = 1};

    void processCollection(std::vector<connection_types::FeaturesMessage> &collection);

};
}

#endif // SVM_ARRAY_TRAINER_H
