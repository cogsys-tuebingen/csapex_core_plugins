/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>

namespace csapex {

/**
 * @brief The Merger class can be used to merge a certain amount of
 *        images.
 */
class EventAND : public csapex::Node
{
public:
    EventAND() :
        caught_a_(false),
        caught_b_(false)
    {
    }

    /**
     * @brief See base class documentation.
     */
    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_a_ = node_modifier.addSlot("A",
                                      std::bind(&EventAND::catchA, this));
        in_b_ = node_modifier.addSlot("B",
                                      std::bind(&EventAND::catchB, this));
        and_event_ = node_modifier.addEvent("AND");
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
    }

    virtual void process() override
    {

    }

private:
    csapex::Slot  *in_a_;
    csapex::Slot  *in_b_;
    csapex::Event *and_event_;
    std::recursive_mutex  m_;
    bool           caught_a_;
    bool           caught_b_;

    void catchA()
    {
        std::unique_lock<std::recursive_mutex> l;
        caught_a_ = true;
        trigger();
    }

    void catchB()
    {
        std::unique_lock<std::recursive_mutex> l;
        caught_b_ = true;
        trigger();
    }

    void trigger()
    {
        if(caught_a_ && caught_b_) {
            msg::trigger(and_event_);
            caught_a_ = false;
            caught_b_ = false;
        }
    }
};
}

CSAPEX_REGISTER_CLASS(csapex::EventAND, csapex::Node)
