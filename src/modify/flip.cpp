/// HEADER
#include "flip.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <boost/assign.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Flip, csapex::Node)

Flip::Flip()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("v", 0)
            ("h", 1)
            ("v+h", -1);
    addParameter(param::ParameterFactory::declareParameterSet("type", types),
                 boost::bind(&Flip::update, this));
}

void Flip::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));
    cv::flip(in->value, out->value, mode_);
    output_->publish(out);
}

void Flip::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    output_ = addOutput<CvMatMessage>("Flipped");
}

void Flip::update()
{
    mode_ = param<int>("type");
}
