#ifndef MLP_NODE_H
#define MLP_NODE_H

/// COMPONENT
#include <csapex_ml/features_message.h>
#include "mlp/mlp.h"

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <mutex>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN MLP : public Node
{
public:
    MLP();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input        *in_;
    Output       *out_;

    std::mutex          m_;
    mlp::MLP::Ptr       mlp_;
    size_t              mlp_input_size_;
    size_t              mlp_output_size_;
    std::vector<int>    mlp_class_labels_;

    void load();

    std::vector<size_t> layers;
    std::vector<double> weights;

    std::vector<std::vector<double>> norm;

    std::string mlp_path;
    std::string norm_path;
};
}

#endif // MLP_NODE_H
