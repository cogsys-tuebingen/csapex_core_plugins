/// HEADER
#include "double_input.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

using namespace csapex;

template <typename T>
NumberInput<T>::NumberInput()
{
}

template <typename T>
void NumberInput<T>::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareValue<T>("value", (T) 0.0));
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("publish"), std::bind(&NumberInput::process, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("tick", false));
}

template <typename T>
void NumberInput<T>::tick()
{
    if(readParameter<bool>("tick")) {
        process();
    }
}

template <typename T>
void NumberInput<T>::setup(NodeModifier& node_modifier)
{
    out_ = node_modifier.addOutput<T>(type2name(typeid(T)));
}

template <typename T>
void NumberInput<T>::process()
{
    T val = readParameter<T>("value");
    msg::publish(out_, val);
}

namespace csapex {
typedef NumberInput<int> IntInput;
typedef NumberInput<double> DoubleInput;
}

CSAPEX_REGISTER_CLASS(csapex::IntInput, csapex::Node)
CSAPEX_REGISTER_CLASS(csapex::DoubleInput, csapex::Node)
