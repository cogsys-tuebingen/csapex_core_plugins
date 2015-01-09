#ifndef RANDOM_TREES_H
#define RANDOM_TREES_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {


class RandomTrees : public csapex::Node
{
public:
    RandomTrees();

    void setupParameters();
    void setup();
    void process();

private:
    void loadTree();
    connection_types::FeaturesMessage classify(const connection_types::FeaturesMessage& input);

private:
    Input* in_;
    Output* out_;

    Slot* reload_;

    cv::RandomTrees dtree_;
    bool loaded_;
};


}

#endif // RANDOM_TREES_H
