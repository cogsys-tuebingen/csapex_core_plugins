
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/no_message.h>
#include <csapex/msg/end_of_program_message.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/param/bitset_parameter.h>
#include <csapex/signal/event.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class MarkerMessageDetector : public Node
{
    enum class Type {
        END_OF_PROGRAM = 4,
        END_OF_SEQUENCE = 2,
        NO_MESSAGE = 1
    };

public:
    MarkerMessageDetector()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");

        event_ = modifier.addEvent("Signal");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        std::map<std::string, std::pair<int, bool> > types;
        types["No Message"] = std::make_pair((int) Type::NO_MESSAGE, true);
        types["End Of Program"] = std::make_pair((int) Type::END_OF_PROGRAM, false);
        types["End Of Sequence"] = std::make_pair((int) Type::END_OF_SEQUENCE, false);

        csapex::param::ParameterPtr p = csapex::param::ParameterFactory::declareParameterBitSet("type", types);
        types_ = std::dynamic_pointer_cast<param::BitSetParameter>(p);

        params.addParameter(types_);
    }

    bool processNoMessageMarkers() const
    {
        return true;
    }

    void processMarker(const connection_types::MessageConstPtr &marker) override
    {
        if(std::dynamic_pointer_cast<NoMessage const>(marker)) {
            if(types_->isSet("No Message")) {
                event_->trigger();
            }

        } else if(std::dynamic_pointer_cast<EndOfSequenceMessage const>(marker)) {
            if(types_->isSet("End Of Sequence")) {
                event_->trigger();
            }

        } else if(std::dynamic_pointer_cast<EndOfProgramMessage const>(marker)) {
            if(types_->isSet("End Of Program")) {
                event_->trigger();
            }
        }
    }

    void process() override
    {
    }

private:
    Input* in_;
    Event* event_;

    param::BitSetParameterPtr types_;
};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::MarkerMessageDetector, csapex::Node)

