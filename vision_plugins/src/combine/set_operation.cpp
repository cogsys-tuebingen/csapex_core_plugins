/// HEADER
#include "set_operation.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/assign/std.hpp>

using namespace vision_plugins;
using namespace csapex;
using namespace connection_types;

CSAPEX_REGISTER_CLASS(vision_plugins::SetOperation, csapex::Node)


using namespace boost::assign;


SetOperation::SetOperation()
{
}

void SetOperation::process()
{
    CvMatMessage::ConstPtr img1 = msg::getMessage<CvMatMessage>(i1_);

    if(!img1->hasChannels(1)) {
        throw std::runtime_error("No Single Channel!");
    }


    CvMatMessage::Ptr out(new CvMatMessage(img1->getEncoding(), img1->stamp_micro_seconds));

    int op = readParameter<int>("operation");
    if(op == COMPLEMENT) {
        out->value = ~img1->value;

    } else {
        if(!msg::hasMessage(i2_)) {
            return;
        }

        CvMatMessage::ConstPtr img2 = msg::getMessage<CvMatMessage>(i2_);

        if(img1->value.rows != img2->value.rows || img1->value.cols != img2->value.cols) {
            throw std::runtime_error("Dimension is not matching!");
        }

        if(op == INTERSECTION) {
            out->value = img1->value & img2->value;

        } else if(op == UNION) {
            out->value = img1->value | img2->value;
        }
    }

    if(out->value.rows != 0 && out->value.cols != 0) {
        msg::publish(out_, out);
    }
}

void SetOperation::setup(NodeModifier& node_modifier)
{
    i1_ = node_modifier.addInput<CvMatMessage>("mask 1");
    i2_ = node_modifier.addOptionalInput<CvMatMessage>("mask 2");
    out_ = node_modifier.addOutput<CvMatMessage>("combined");
}

void SetOperation::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> methods = map_list_of
            ("Complement", (int) COMPLEMENT)
            ("Intersection", (int) INTERSECTION)
            ("Union", (int) UNION);

    parameters.addParameter(param::ParameterFactory::declareParameterSet("operation", methods, (int) UNION));
}
