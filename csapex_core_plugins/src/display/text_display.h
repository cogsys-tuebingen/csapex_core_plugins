#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex_core_plugins_node_export.h>
#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_CORE_PLUGINS_NODE_EXPORT TextDisplay : public Node
{
public:
    TextDisplay();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;

public:
    slim_signal::Signal<void(const std::string&)> display_request;

protected:
    void display(const TokenDataConstPtr msg);
    void convert(std::stringstream& ss, const YAML::Node& node, const std::string& prefix);

private:
    Input* input_;
    Slot* slot_;
};

}  // namespace csapex

#endif  // TEXT_DISPLAY_H_
