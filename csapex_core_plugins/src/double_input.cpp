/// HEADER
#include "double_input.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;

template <typename T>
NumberInput<T>::NumberInput()
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("General"));

    addParameter(param::ParameterFactory::declareValue<T>("value", (T) 0.0));
    addParameter(param::ParameterFactory::declareTrigger("publish"), boost::bind(&NumberInput::process, this));
    addParameter(param::ParameterFactory::declareBool("tick", false));
}

template <typename T>
void NumberInput<T>::tick()
{
    if(readParameter<bool>("tick")) {
        process();
    }
}

template <typename T>
QIcon NumberInput<T>::getIcon() const
{
    return QIcon(":/pencil.png");
}

template <typename T>
void NumberInput<T>::setup()
{
    out_ = modifier_->addOutput<T>(type2name(typeid(T)));
}

template <typename T>
void NumberInput<T>::process()
{
    T val = readParameter<T>("value");
    out_->publishIntegral(val);
}

namespace csapex {
typedef NumberInput<int> IntInput;
typedef NumberInput<double> DoubleInput;
}

CSAPEX_REGISTER_CLASS(csapex::IntInput, csapex::Node)
CSAPEX_REGISTER_CLASS(csapex::DoubleInput, csapex::Node)
