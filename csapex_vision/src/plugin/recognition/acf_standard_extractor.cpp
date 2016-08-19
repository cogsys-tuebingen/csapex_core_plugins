/// HEADER
#include "acf_standard_extractor.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_ml/features_message.h>

using namespace csapex;
using namespace connection_types;


void ACFStandardExtractor::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("/window_width",
                                                                  10, 1024, 64, 1),
                            std::bind(&ACFStandardExtractor::update, this));
    parameters.addParameter(param::ParameterFactory::declareRange("/window/height",
                                                                  10, 1024, 128, 1),
                            std::bind(&ACFStandardExtractor::update, this));
    parameters.addParameter(param::ParameterFactory::declareBool("/window/mirror",
                                                                 false),
                            mirror_);
    parameters.addParameter(param::ParameterFactory::declareBool("/window/keep_ratio",
                                                                 false),
                            keep_ratio_);
}

void ACFStandardExtractor::setup(NodeModifier &node_modifier)
{

}

void ACFStandardExtractor::process()
{



}

void ACFStandardExtractor::update()
{

}
