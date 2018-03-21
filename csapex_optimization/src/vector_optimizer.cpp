/// HEADER
#include <csapex_optimization/vector_optimizer.h>
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

using namespace csapex;
using namespace csapex::connection_types;

VectorOptimizer::VectorOptimizer():
    is_running_(false),
    has_new_parameter_(false),
    has_residual_(false),
    fitness_calls_(0),
    residual_(std::numeric_limits<double>::infinity())
{
}

void VectorOptimizer::setup(csapex::NodeModifier& modifier)
{
    in_ = modifier.addSlot<GenericValueMessage<double>>("residual", [this](const TokenPtr& residual){
        residualCb(residual);
    });

    in_grad_ = modifier.addSlot<GenericVectorMessage>("gradient", [this](const TokenPtr& grad){
        gradCb(grad);
    });

    in_start_ = modifier.addSlot<GenericVectorMessage>("start params", [this](const TokenPtr& msg){
        startParamCb(msg);
    });

    out_ = modifier.addOutput<GenericVectorMessage, double>("parameter");
    evaluate_ = modifier.addEvent("evaluate");
    solution_ = modifier.addEvent<GenericVectorMessage, double>("min parameters");
    min_residual_ = modifier.addEvent<double>("min residual");
}

void VectorOptimizer::setupParameters(csapex::Parameterizable& params)
{

    params.addParameter(param::ParameterFactory::declareTrigger("restart"),[this](param::Parameter*){
        start();
    });

    params.addParameter(param::ParameterFactory::declareTrigger("stop"),[this](param::Parameter*){
        stop();
    });

    params.addParameter(param::ParameterFactory::declareBool("trigger_immediately",false),
                        trigger_immediately_);

    params.addParameter(param::ParameterFactory::declareRange("polling_time",
                                                              param::ParameterDescription("Time in ms after which is checked if data has arrived."),
                                                              1,1000,20,1),
                        wait_time_ms_);

    params_.setupParameters(params);
}

void VectorOptimizer::publishOptimalParameters()
{
    GenericVectorMessage::Ptr msg_para(GenericVectorMessage::make<double>());
    std::shared_ptr<std::vector<double>> out_msg(new std::vector<double>);
    out_msg->insert(out_msg->begin(),to_optimize_.begin(), to_optimize_.end());
    msg_para->set<double>(out_msg);
    solution_->triggerWith(std::make_shared<Token>(msg_para));
}
void VectorOptimizer::publishMinimumResidual()
{
    // publish minimum residual
    GenericValueMessage<double>::Ptr msg_res(new GenericValueMessage<double>);
    msg_res->value = minimum_residual_;
    min_residual_->triggerWith(std::make_shared<Token>(msg_res));
    ext_start_params_.clear();
}

void VectorOptimizer::setDone()
{
    is_running_ = false;
    has_residual_ = false;
    has_grad_ = false;
    ext_start_params_.clear();
}

void VectorOptimizer::createStart()
{
    std::unique_lock<std::mutex> lock(data_available_mutex_);
    has_residual_ = false;
    has_grad_ = false;
    to_optimize_.clear();
    if(ext_start_params_.empty()){
        if(params_.useStart()){
            to_optimize_ = params_.getStartParams();
        }
        else{
            to_optimize_ = params_.getRandomStart();
        }
    }
    else{
        to_optimize_ = ext_start_params_;
    }
    evaluate_->trigger();
    has_new_parameter_ = true;
    residual_ = std::numeric_limits<double>::quiet_NaN();
    yield();
    while(is_running_ && !has_residual_){
        data_available_.wait_for(lock, std::chrono::milliseconds(500));
    }
}

void VectorOptimizer::tearDown()
{
    is_running_ = false;
    data_available_.notify_all();
}

bool VectorOptimizer::canProcess() const
{
    return is_running_ && has_new_parameter_;
}

void VectorOptimizer::process()
{
    std::shared_ptr<std::vector<double>> msg = std::make_shared<std::vector<double>>(to_optimize_);

    msg::publish<GenericVectorMessage, double>(out_, msg);
    has_new_parameter_ = false;
}

void VectorOptimizer::residualCb(const TokenPtr &value)
{
    if(auto val = std::dynamic_pointer_cast<GenericValueMessage<double> const>(value->getTokenData())){
        std::unique_lock<std::mutex> lock(data_available_mutex_);
        residual_ = val->value;
        lock.unlock();
        if(trigger_immediately_){
            has_residual_ = true;
            nextParams();
        }
    }
    else if(std::dynamic_pointer_cast<EndOfSequenceMessage const>(value->getTokenData())){
        if(!trigger_immediately_){
            has_residual_ = true;
            nextParams();
        }
    }
    else{
        throw std::runtime_error("unkown message recieved: " + value->getTokenData()->typeName());
    }
}

void VectorOptimizer::gradCb(const TokenPtr &grad)
{
    if(auto val = std::dynamic_pointer_cast<GenericVectorMessage const>(grad->getTokenData())){
        std::unique_lock<std::mutex> lock(data_available_mutex_);
        std::size_t size = val->nestedValueCount();
        gradient_.resize(size);
        auto it = gradient_.begin();
        for(std::size_t i = 0; i < size; ++i){
            auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(val->nestedValue(i));
            *it = pval->value;
            ++it;
        }
        lock.unlock();
        if(trigger_immediately_){
            has_grad_ = true;
            nextParams();
        }
    }
    else if(std::dynamic_pointer_cast<EndOfSequenceMessage const>(grad->getTokenData())){
        if(!trigger_immediately_){
            has_grad_ = true;
            nextParams();
        }
    }
    else{
        throw std::runtime_error("unkown message recieved: " + grad->getTokenData()->typeName());
    }
}

void VectorOptimizer::startParamCb(const TokenPtr &msg)
{
    if(auto val = std::dynamic_pointer_cast<GenericVectorMessage const>(msg->getTokenData())){
        std::unique_lock<std::mutex> lock(data_available_mutex_);
        std::size_t size = val->nestedValueCount();
        ext_start_params_.resize(size);
        auto it = ext_start_params_.begin();
        for(std::size_t i = 0; i < size; ++i){
            auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(val->nestedValue(i));
            *it = pval->value;
            ++it;
        }
        params_.setStartParams(ext_start_params_);
        lock.unlock();
    }
    else if(std::dynamic_pointer_cast<EndOfSequenceMessage const>(msg->getTokenData())){

    }
    else{
        throw std::runtime_error("unkown message recieved: " + msg->getTokenData()->typeName());
    }


}


void VectorOptimizer::nextParams()
{
    std::unique_lock<std::mutex> lock(data_available_mutex_);
    if(!canNotifiy()){
        return;
    }
    if(!std::isnormal(residual_)){
        residual_ = std::numeric_limits<double>::infinity();
    }

    data_available_.notify_all();
}

bool VectorOptimizer::canNotifiy()
{
    return has_residual_ && (has_grad_ || use_grad_);
}

double VectorOptimizer::realMinfunc(const std::vector<double> &x, std::vector<double> &grad)
{
    std::unique_lock<std::mutex> lock(data_available_mutex_);
    has_residual_ = false;
    has_grad_ = false;
    evaluate_->trigger();
    to_optimize_ = x;
    has_new_parameter_ = true;
    residual_ = std::numeric_limits<double>::quiet_NaN();
    yield();
    while(is_running_ && (!has_residual_  || (!grad.empty() && !has_grad_))){
        data_available_.wait_for(lock, std::chrono::milliseconds(wait_time_ms_));
    }
    if(!is_running_){
        return std::numeric_limits<double>::infinity();
    }
    if(!grad.empty()){
        apex_assert(grad.size() ==  gradient_.size());
        grad = gradient_;
    }
    //        ainfo << "residual: " << residual_ << std::endl;
    ++fitness_calls_;
    return residual_;
}
