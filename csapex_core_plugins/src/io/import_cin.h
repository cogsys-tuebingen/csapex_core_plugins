#ifndef IMPORT_CIN_H
#define IMPORT_CIN_H

/// PROJECT
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <deque>

namespace csapex {

class ImportCin : public TickableNode
{
public:
    ImportCin();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void tick() override;

private:
    void readCin();
    void publishNextMessage();
    void readMessages();

private:
    Output* connector_;

    std::stringstream buffer;

    std::deque<ConnectionTypeConstPtr> message_buffer_;
};

}

#endif // IMPORT_CIN_H
