
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/signal/event.h>
#include <csapex/model/token.h>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class GetBestOptimizationResult : public Node
{
public:
    GetBestOptimizationResult()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<double>("function value");
        in_params_ = modifier.addInput<AnyMessage>("parameter");
        out_best_ = modifier.addEvent<AnyMessage>("best parameter");
//        out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareTrigger("clear"),[this](param::Parameter*){
            opt_results_.clear();
        });
        params.addParameter(param::ParameterFactory::declareTrigger("get_best"),[this](param::Parameter*){

            auto it = std::min_element(opt_results_.begin(), opt_results_.end(),
                                       [](std::pair<double, TokenData::ConstPtr>& l,
                                          std::pair<double, TokenData::ConstPtr>& r) -> bool { return l.first < r.first; });


            TokenPtr token = std::make_shared<Token>(it->second);
            out_best_->triggerWith(token);
        });

    }

    void process() override
    {
        auto value = msg::getValue<double>(in_);
        TokenData::ConstPtr m =  msg::getMessage(in_params_);

        std::pair<double, TokenData::ConstPtr> p;
        p.first = value;

        opt_results_.push_back(p);
//        msg::publish(out_, value + "!");
    }

private:
    Input* in_;
    Input* in_params_;
    Output* out_;
    Event* out_best_;
    std::vector<std::pair<double, TokenData::ConstPtr>> opt_results_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::GetBestOptimizationResult, csapex::Node)

