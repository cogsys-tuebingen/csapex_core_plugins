#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TextDisplay : public Node
{
public:
    TextDisplay();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

public:
    csapex::slim_signal::Signal<void(const std::string&)> display_request;

protected:
    void display(const TokenConstPtr msg);
    void convert(std::stringstream& ss, const YAML::Node& node, const std::string &prefix);

private:
    Input* input_;
    Slot* slot_;
};

}

#endif // TEXT_DISPLAY_H_
