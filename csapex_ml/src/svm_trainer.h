#ifndef SVM_TRAINER_H
#define SVM_TRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <mutex>

namespace csapex {
class SVMTrainer : public Node
{
public:
    typedef csapex::connection_types::FeaturesMessage::ConstPtr FeaturePtr;

    SVMTrainer();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input*                       in_vector_;

    std::size_t                  step_;
    std::vector<FeaturePtr>      msgs_;
    std::string  path_;
    bool         save_for_hog_;
    cv::SVMParams svm_params_;

    void train();
    void clear();
};
}

#endif // SVM_TRAINER_H
