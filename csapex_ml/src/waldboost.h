#ifndef WALDBOOST_H
#define WALDBOOST_H

/// PROJECT
#include "waldboost/waldboost.hpp"
#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN WaldBoost : public Node
{
public:
    WaldBoost();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    Input* in_;
    Output* out_;
    Slot* reload_;

    cv::WaldBoost wb_;
    std::string path_;
    bool loaded_;

    void reload();
};
}  // namespace csapex

#endif  // WALDBOOST_H
