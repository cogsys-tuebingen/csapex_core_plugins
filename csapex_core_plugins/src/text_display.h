#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TextDisplay : public Node
{
public:
    TextDisplay();

    virtual void process();
    virtual void setup();

public:
    boost::signals2::signal<void(const std::string&)> display_request;

private:
    void convert(std::stringstream& ss, const YAML::Node& node, const std::string &prefix);

private:
    Input* connector_;
};

}

#endif // TEXT_DISPLAY_H_
