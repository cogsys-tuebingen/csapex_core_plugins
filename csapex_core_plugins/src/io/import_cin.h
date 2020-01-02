#ifndef IMPORT_CIN_H
#define IMPORT_CIN_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <deque>
#include <sstream>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN ImportCin : public Node
{
public:
    ImportCin();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& params) override;
    void process() override;

private:
    void readCin();
    void publishNextMessage();
    void readMessages();
    void readYAML(const std::string& message);

private:
    Output* connector_;

    std::stringstream buffer;

    std::deque<TokenDataConstPtr> message_buffer_;
    TokenDataConstPtr last_message_;

    bool import_yaml_;
    bool latch_;
    bool signal_end_;
};

}  // namespace csapex

#endif  // IMPORT_CIN_H
