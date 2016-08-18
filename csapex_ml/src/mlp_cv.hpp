#pragma once

#include <csapex/model/node.h>
#include <csapex_ml/features_message.h>

#include <opencv2/ml/ml.hpp>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN  MLPCv : public Node
{
public:
    MLPCv();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    void classify(const connection_types::FeaturesMessage &input, connection_types::FeaturesMessage &output);
    void loadMLP();

private:
    Input* input_;
    Output* output_;
    Slot* reload_;

    cv::NeuralNet_MLP mlp_;
    bool loaded_;
};

}
