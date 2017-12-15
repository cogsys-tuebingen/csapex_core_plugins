#include <csapex_optimization/optimization_params.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/value_parameter.h>
#include <map>
#include <iostream>
#include <random>
using namespace csapex;

OptimizationParams::OptimizationParams() :
    set_bounds_(true),
    problem_dim_(1)
{

}

void OptimizationParams::setFormExp(double &tar, param::Parameter* p)
{
    int exp = p->as<int>();
    tar = std::pow(10.0,exp);
}


void OptimizationParams::setupParameters(csapex::Parameterizable& params)
{

    params_ = &params;

    params.addParameter(param::ParameterFactory::declareRange("problem_dimension",1,100,1,1),
                        [this](param::Parameter* p){
        double new_size = (std::size_t) p->as<int>();
        if(new_size < problem_dim_){
            removeBounds(new_size);
            removeStart(new_size);
        }
        problem_dim_ = new_size;
        lower_bounds_.resize(problem_dim_);
        upper_bounds_.resize(problem_dim_);
        start_values_.resize(problem_dim_);
        boundParams();
        startParams();
    });

    params.addParameter(param::ParameterFactory::declareBool("set_bounds",false),
                        [this](param::Parameter* p){
        set_bounds_ = p->as<bool>();
        if(set_bounds_){
            boundParams();
        }
    });



    params.addConditionalParameter(param::ParameterFactory::declareBool("bounds/relative",false),
                                   set_bounds_,relative_bounds_);

    params.addConditionalParameter(param::ParameterFactory::declareBool("bounds/one_bound_for_all_params",false),
                                   set_bounds_,
                                   [this](param::Parameter* p){
        one_bound_for_all_ = p->as<bool>();
        if(one_bound_for_all_){
            removeBounds(1);
        }
        else{
            boundParams();
        }
    });

    params.addParameter(param::ParameterFactory::declareBool("set_start",false),
                        [this](param::Parameter* p){
        set_start_ = p->as<bool>();
        if(set_start_){
            startParams();
        }
    });
}

void OptimizationParams::getBounds(std::vector<double>& lower, std::vector<double>& upper) const
{
    if(set_bounds_){
        lower.clear();
        upper.clear();
        if(relative_bounds_){
            apex_assert_eq(start_values_.size(), problem_dim_);
            auto it_low = lower_bounds_.begin();
            auto it_up = upper_bounds_.begin();
            for(auto val : start_values_){
                double l = (1 + *it_low) * val;
                double u = (1 + *it_up) * val;
                if(std::abs(val) < 1e-7){
                    lower.emplace_back(*it_low);
                    upper.emplace_back(*it_up);
                }
                else{
                    lower.emplace_back(std::min(l,u));
                    upper.emplace_back(std::max(l,u));
                }
                if(!one_bound_for_all_){
                    ++it_low;
                    ++it_up;
                }
            }
        }
        else{
            for(std::size_t i = 0; i < problem_dim_; ++i){
                std::pair<double, double> bounds;
                if(one_bound_for_all_){
                    bounds = params_->readParameter<std::pair<double, double>>("~bounds/interval_0");
                } else {
                    bounds = params_->readParameter<std::pair<double, double>>("~bounds/interval_" + std::to_string(i));
                }
                lower.emplace_back(bounds.first);
                upper.emplace_back(bounds.second);
            }
        }
    }
}

void  OptimizationParams::boundParams()
{

    if(set_bounds_){
        std::size_t max_dim = problem_dim_;
        if(one_bound_for_all_){
            max_dim = 1;
            removeBounds(max_dim);
        }
        for(std::size_t i = 0; i < max_dim; ++i){
            param::Parameter::Ptr intervall =
                    param::ParameterFactory::declareInterval("~bounds/interval_" + std::to_string(i),
                                                             -10.0,
                                                             10.0,
                                                             -10.0,
                                                             10.0,
                                                             0.01);
            //            std::function<void(param::Parameter* p)> cb = [this, i](param::Parameter* p){
            //                if(p->is<std::pair<double,double>>()){
            //                    p->as
            //                    std::pair<double, double> bound = p->as<std::pair<double,double>>();
            //                    std::cout <<"intervall: "<< i << " " << bound.first << " " << bound.second << std::endl;
            //                    lower_bounds_[i] = bound.first;
            //                    upper_bounds_[i] = bound.second;

            //                }
            //            };
            params_->addTemporaryParameter(intervall);
            //            params_->addParameterCallback(intervall, cb);
        }
    }
    else{
        removeBounds(0);
    }
}

void  OptimizationParams::startParams()
{

    if(set_start_){
        for(std::size_t i = 0; i < problem_dim_; ++i){
            param::Parameter::Ptr startVal =
                    param::ParameterFactory::declareValue("~start/x0_" +std::to_string(i),
                                                          0.1);

            std::function<void(param::Parameter* p)> cb = [this, i](param::Parameter* p){
                if(p->is<double>()){
                    start_values_[i] = p->as<double>();

                }
            };
            params_->addTemporaryParameter(startVal);
            params_->addParameterCallback(startVal, cb);
        }
    }
    else{
        removeStart(0);
    }
}

void OptimizationParams::removeBounds(std::size_t new_size)
{
    for(int i = ((int) problem_dim_) -1; i >= (int) new_size; --i){


        param::Parameter::Ptr intervall;
        if(relative_bounds_){
            intervall = param::ParameterFactory::declareInterval("~bounds/interval_" +std::to_string(i),
                                                                 -10.0,
                                                                 10.0,
                                                                 -0.1,
                                                                 0.1,
                                                                 0.1);
        }
        else{
            intervall = param::ParameterFactory::declareInterval("~bounds/interval_" +std::to_string(i),
                                                                 -100.0,
                                                                 100.0,
                                                                 -100.0,
                                                                 100.0,
                                                                 1.0);
        }

        params_->removeTemporaryParameter(intervall);


    }
}

void OptimizationParams::removeStart(std::size_t new_size)
{
    for(int i = ((int) problem_dim_) -1; i >= (int) new_size; --i){


        param::Parameter::Ptr val =
                param::ParameterFactory::declareValue("~start/x0_" +std::to_string(i),
                                                      0.1);
        params_->removePersistentParameter(val);


    }
}

std::vector<double> OptimizationParams::getRandomStart() const
{
    std::vector<double> res;
    std::default_random_engine re;
    for(std::size_t i = 0; i < problem_dim_; ++i){
        std::uniform_real_distribution<double> unif;
        if(set_bounds_){
            std::pair<double, double> b;
            if(one_bound_for_all_){
                b = params_->readParameter<std::pair<double, double>>("~bounds/interval_0");
            } else{
                b  = params_->readParameter<std::pair<double, double>>("~bounds/interval_" + std::to_string(i));
            }
            unif = std::uniform_real_distribution<double>(b.first, b.second);
        }
        else{
            unif = std::uniform_real_distribution<double>(std::numeric_limits<double>::min(), std::numeric_limits<double>::max());
        }
        double random_double = unif(re);
        res.emplace_back(random_double);
    }
    return res;
}

void OptimizationParams::checkBounds()
{
    auto it_l = lower_bounds_.begin();
    for(double& u : upper_bounds_){
        double tmp_l = *it_l;
        double tmp_u = u;
        if(tmp_u < tmp_l){
            *it_l = std::min(tmp_u, tmp_l);
            u = std::max(tmp_u, tmp_l);
        }
        ++it_l;
    }
}
