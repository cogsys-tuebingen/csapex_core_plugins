
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/algorithm/string.hpp>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class ReplaceString : public Node
{
public:
    ReplaceString()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<std::string>("Input");
        out_ = modifier.addOutput<std::string>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareText("needle", ""), needle_);
        params.addParameter(param::factory::declareText("replacement", ""), replacement_);
    }

    void process() override
    {
        std::string value = msg::getValue<std::string>(in_);

        if (!needle_.empty()) {
            boost::replace_all(value, needle_, replacement_);
        }

        msg::publish(out_, value);
    }

private:
    Input* in_;
    Output* out_;

    std::string needle_;
    std::string replacement_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ReplaceString, csapex::Node)
