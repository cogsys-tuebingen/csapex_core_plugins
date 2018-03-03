
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/signal/event.h>
#include <csapex/model/token.h>
#include <csapex/signal/slot.h>
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
        out_best_ = modifier.addEvent<GenericVectorMessage, double>("best parameter");

        in_ = modifier.addSlot<GenericValueMessage<double>>("fitness", [this](const TokenPtr& data){
            fitnessCb(data);
        });

        in_params_ = modifier.addSlot<GenericVectorMessage, double>("parameter", [this](const TokenPtr& msg){
            paramCb(msg);
        });

        //        out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareTrigger("clear"),[this](param::Parameter*){
            fitness_.clear();
            params_.clear();
        });
        params.addParameter(param::ParameterFactory::declareTrigger("get_best"),[this](param::Parameter*){
            std::stringstream str;
            str << "It is expected to have the same number of fitness values and paramter messages."
                << " Got " << fitness_.size() << " fitness values and "   << params_.size() << " parameter messages.";
            apex_assert_msg(fitness_.size() == params_.size(),str.str());

            auto it = std::min_element(fitness_.begin(), fitness_.end());
            std::size_t id = it - fitness_.begin();


            TokenPtr token = std::make_shared<Token>(params_.at(id));
            out_best_->triggerWith(token);
        });

    }

    void fitnessCb(const TokenPtr &data)
    {
        if(auto val = std::dynamic_pointer_cast<GenericValueMessage<double> const>(data->getTokenData())){
//            std::unique_lock<std::mutex> lock(data_available_mutex_);
            fitness_.push_back(val->value);

        }
        else if(std::dynamic_pointer_cast<EndOfSequenceMessage const>(data->getTokenData())){
        }
        else{
            throw std::runtime_error("unkown message recieved: " + data->getTokenData()->typeName());
        }
    }

    void paramCb(const TokenPtr &msg)
    {
        if(std::dynamic_pointer_cast<EndOfSequenceMessage const>(msg->getTokenData())){
            return;
        }
        params_.push_back(msg->getTokenData());
    }

    void process() override
    {
//        auto value = msg::getValue<double>(in_);
//        TokenData::ConstPtr m =  msg::getMessage(in_params_);

//        std::pair<double, TokenData::ConstPtr> p;
//        p.first = value;

//        opt_results_.push_back(p);
        //        msg::publish(out_, value + "!");
    }

private:
    Slot* in_;
    Slot* in_params_;
    Output* out_;
    Event* out_best_;
    std::vector<double> fitness_;
    std::vector<TokenData::ConstPtr> params_;
    std::mutex data_available_mutex_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::GetBestOptimizationResult, csapex::Node)

