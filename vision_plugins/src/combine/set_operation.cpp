/// HEADER
#include "set_operation.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
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
    CvMatMessage::Ptr img1 = i1_->getMessage<CvMatMessage>();

    if(img1->getEncoding() != enc::mono) {
        throw std::runtime_error("No Single Channel!");
    }


    CvMatMessage::Ptr out(new CvMatMessage(img1->getEncoding()));

    int op = readParameter<int>("operation");
    if(op == COMPLEMENT) {
        out->value = ~img1->value;

    } else {
        if(!i2_->isConnected() || !i2_->hasMessage()) {
            return;
        }

        CvMatMessage::Ptr img2 = i2_->getMessage<CvMatMessage>();

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
        out_->publish(out);
    }
}

void SetOperation::setup()
{
    i1_ = modifier_->addInput<CvMatMessage>("mask 1");
    i2_ = modifier_->addInput<CvMatMessage>("mask 2", true);
    out_ = modifier_->addOutput<CvMatMessage>("combined");
}

void SetOperation::setupParameters()
{
    std::map<std::string, int> methods = map_list_of
            ("Complement", (int) COMPLEMENT)
            ("Intersection", (int) INTERSECTION)
            ("Union", (int) UNION);

    addParameter(param::ParameterFactory::declareParameterSet("operation", methods, (int) UNION));
}
