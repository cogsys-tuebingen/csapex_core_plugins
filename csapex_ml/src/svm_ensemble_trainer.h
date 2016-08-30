#ifndef SVM_ARRAY_TRAINER_H
#define SVM_ARRAY_TRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex_core_plugins/collection_node.h>
#include "random_vector.hpp"

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN SVMEnsembleTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    SVMEnsembleTrainer();

    void setup(NodeModifier &modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    std::string     path_;
    bool            save_for_hog_;
    bool            one_vs_all_;
    bool            balance_;
    cv::SVMParams   svm_params_;


    enum ClassTypes {NEGATIVE = -1, POSITIVE = 1};

    bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;
    RandomVector rand_vec_;

};
}

#endif // SVM_ARRAY_TRAINER_H
