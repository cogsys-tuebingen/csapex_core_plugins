/// PROJECT
#include <csapex/model/node.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/param/range_parameter.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/msg/end_of_sequence_message.h>

using namespace csapex;

namespace csapex {

template <typename T>
class NumberRangeGenerator : public Node
{
public:
    NumberRangeGenerator()
        : active_(true), end_reached_(false)
    {

    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        out_ = node_modifier.addOutput<T>(type2name(typeid(T)));
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        param_  = param::ParameterFactory::declareRange<T>("range",
                                                           param::ParameterDescription("Adjust the properties of this parameter to your needs."),
                                                           (T) 0.0, (T) 10.0, (T) 10.0, (T) 1.0).
                template instantiate<param::RangeParameter>();
        parameters.addParameter(param_);

        param::TriggerParameter::Ptr reset = param::ParameterFactory::declareTrigger("reset").
                template instantiate<param::TriggerParameter>();

        reset->first_connect.connect([this](param::Parameter*) {
            active_ = false;
        });
        reset->last_disconnect.connect([this](param::Parameter*) {
            active_ = true;
        });
        parameters.addParameter(reset, [this](param::Parameter*){
            this->reset();
            active_ = true;
            yield();
        });
    }

    bool canProcess() const override
    {
        return active_;
    }

    void process() override
    {
        if(end_reached_) {
            msg::publish(out_, makeEmpty<connection_types::EndOfSequenceMessage>());
            active_ = false;
            return;
        }

        msg::publish(out_, val_);

        val_ += param_->step<T>();
        param_->set<T>(val_);

        if(val_ > param_->max<T>()) {
            val_ = param_->max<T>();
            end_reached_ = true;
        }
    }

    void reset() override
    {
        val_ = param_->min<T>();
        end_reached_ = false;

        active_ = true;

        yield();
    }

private:
    Output* out_;

    param::RangeParameterPtr param_;

    T val_;

    bool active_;
    bool end_reached_;
};

}

namespace csapex {
typedef NumberRangeGenerator<int> IntRangeGenerator;
typedef NumberRangeGenerator<double> DoubleRangeGenerator;
}

CSAPEX_REGISTER_CLASS(csapex::IntRangeGenerator, csapex::Node)
CSAPEX_REGISTER_CLASS(csapex::DoubleRangeGenerator, csapex::Node)
