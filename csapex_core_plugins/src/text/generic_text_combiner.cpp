/// HEADER
#include "generic_text_combiner.h"

/// PROJECT
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/assert.h>

CSAPEX_REGISTER_CLASS(csapex::GenericTextCombiner, csapex::Node)

using namespace csapex;
using namespace connection_types;

GenericTextCombiner::GenericTextCombiner()
{
}

void GenericTextCombiner::updateFormula()
{
    format = readParameter<std::string>("format");
}

void GenericTextCombiner::process()
{
    std::vector<Input*> inputs = node_modifier_->getMessageInputs();
    if(inputs.empty()) {
        return;
    }

    std::stringstream ss;

    for(std::size_t i = 0, n = format.size(); i < n;) {
        if(format[i] == '$' && (i == 0 || format[i-1] != '\\'))
        {
            // variable
            ++i;
            std::string num;
            while(i < n && format[i] >= '0' && format[i] <= '9') {
                num += format[i];
                ++i;
            }
            int param_num = atoi(num.c_str());

            Input* i = inputs.at(param_num-1);
            if(msg::isValue<std::string>(i)) {
                ss << msg::getValue<std::string>(i);

            } else if(msg::isValue<int>(i)) {
                ss << msg::getValue<int>(i);

            } else if(msg::isValue<double>(i)) {
                ss << msg::getValue<double>(i);

            } else {
                ss << "[unknown]";
            }

        } else {
            ss << format[i];
            ++i;
        }

    }

    setParameter("last_value", std::string("last result: ") + ss.str());
    msg::publish(out_, ss.str());
}

void GenericTextCombiner::setup(NodeModifier& node_modifier)
{
    setupVariadic(node_modifier);

    out_ = node_modifier.addOutput<std::string>("combined");
}

void GenericTextCombiner::setupParameters(Parameterizable& parameters)
{
    setupVariadicParameters(parameters);

    parameters.addParameter(param::ParameterFactory::declareText("format",
                                                                 param::ParameterDescription("A format string. Inputs are inserted for $1,$2, ...$n"),
                                                                 "$1_$2"),
                            std::bind(&GenericTextCombiner::updateFormula, this));
    parameters.addParameter(param::ParameterFactory::declareOutputText("last_value"));
}

Input* GenericTextCombiner::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool /*optional*/)
{
    return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
}
