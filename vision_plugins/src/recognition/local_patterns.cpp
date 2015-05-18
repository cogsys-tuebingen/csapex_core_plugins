/// HEADER
#include "local_patterns.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>

#include <csapex/model/connection_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>

#include <utils_vision/textures/lbp.hpp>
#include <utils_vision/textures/ltp.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::LocalPatterns, csapex::Node)

using namespace csapex;
using namespace vision_plugins;
using namespace connection_types;


LocalPatterns::LocalPatterns()
{

}

void LocalPatterns::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(in_);
    CvMatMessage::Ptr out(new CvMatMessage(enc::mono, in->stamp_micro_seconds));

    if(in->value.channels() > 1)
        throw std::runtime_error("Matrix must be one channel!");

    double k = readParameter<double>("k1");
    Type   t = (Type) readParameter<int>("pattern");

    switch(t) {
    case LBP:
        utils_vision::LBP::standard(in->value, k, out->value);
        break;
    case LTP:
        utils_vision::LTP::standard(in->value, k, out->value);
        out->setEncoding(enc::unknown);
        break;
    default:
        throw std::runtime_error("Unknown local pattern!");
    }

    msg::publish(out_, out);
}

void LocalPatterns::setup(NodeModifier &node_modifier)
{
    out_ = node_modifier.addOutput<CvMatMessage>("Clusters");
    in_  = node_modifier.addInput<CvMatMessage>("Mono");
}

void LocalPatterns::setupParameters(Parameterizable &parameters)
{
    std::map<std::string, int> types =
            boost::assign::map_list_of
            ("LBP", LBP)("LTP", LTP);

    param::Parameter::Ptr type =
            param::ParameterFactory::declareParameterSet("pattern",
                                                         types,
                                                         (int) LBP);
    std::function<bool()> condition = [type]() { return type->as<int>() == LTP || type->as<int>() == LBP; };

    parameters.addParameter(type);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("k1", -100.0, 100.0, 0.0, 0.1),
                            condition);

}
