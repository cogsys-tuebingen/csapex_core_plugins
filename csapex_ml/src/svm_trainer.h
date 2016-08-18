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

    void setupParameters(Parameterizable& parameters);

private:
    std::string     path_;
    bool            save_for_hog_;
    cv::SVMParams   svm_params_;

    void processCollection(std::vector<connection_types::FeaturesMessage> &collection);

};
}

#endif // SVM_TRAINER_H
