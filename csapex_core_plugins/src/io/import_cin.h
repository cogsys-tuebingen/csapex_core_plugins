#ifndef IMPORT_CIN_H
#define IMPORT_CIN_H

/// PROJECT
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <deque>
#include <sstream>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN ImportCin : public TickableNode
{
public:
    ImportCin();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& params) override;
    virtual void tick() override;

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
};

}

#endif // IMPORT_CIN_H
