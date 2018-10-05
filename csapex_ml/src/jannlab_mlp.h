#ifndef MLP_NODE_H
#define MLP_NODE_H

/// COMPONENT
#include "mlp/mlp.h"
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <mutex>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN JANNLabMLP : public Node
{
public:
    JANNLabMLP();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input* in_;
    Output* out_;

    std::mutex m_;
    mlp::MLPConfig config_;
    mlp::MLP::Ptr mlp_;
    size_t mlp_input_size_;
    size_t mlp_output_size_;
    std::vector<int> mlp_class_labels_;

    void load();
    void loadNorm();
    void loadClassLabels();

    //    std::vector<size_t> layers_;
    //    std::vector<double> weights_;

    std::vector<std::vector<double>> norm_;

    std::string mlp_path_;
    std::string class_label_path_;
    std::string norm_path_;
};
}  // namespace csapex

#endif  // MLP_NODE_H
