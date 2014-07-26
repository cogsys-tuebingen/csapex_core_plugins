/// HEADER
#include "hog_training_rois.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::HOGTrainingRois, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


HOGTrainingRois::HOGTrainingRois()
{
}

void HOGTrainingRois::setupParameters()
{
    // addParameter(param::ParameterFactory::declareRange("range", 0.01, 1.0, 0.2, 0.01), update);

    //std::map<std::string, int> featuremix = boost::assign::map_list_of
    //        ("standard", 0)
    //        ("no foo", 1)
    //        ("no bar", 2);

    // addParameter(param::ParameterFactory::declareParameterSet<int>("feature mix", featuremix));
}

void HOGTrainingRois::setup()
{
    //in_  = modifier_->addInput<TYPE>(LABEL);
    //out_ = modifier_->addOutput<TYPE>(LABEL);
}

void HOGTrainingRois::process()
{
}

