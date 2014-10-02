#ifndef SVM_TRAINER_H
#define SVM_TRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/thread/mutex.hpp>

namespace csapex {
class SVMTrainer : public Node
{
public:
    SVMTrainer();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();

private:
    Input*                                              in_;
    Input*                                              in_vector_;

    boost::mutex                                        m_;
    unsigned int                                        step_;
    std::vector<connection_types::FeaturesMessage::Ptr> msgs_;

    void train();
    void clear();
};
}

#endif // SVM_TRAINER_H
