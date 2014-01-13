/// HEADER
#include "combiner_set_operation.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QComboBox>

CSAPEX_REGISTER_CLASS(csapex::SetOperation, csapex::Node)

using namespace csapex;
using namespace connection_types;

SetOperation::SetOperation()
{
    addTag(Tag::get("Vision"));

    std::vector< std::pair<std::string, int> > methods;
    methods.push_back(std::make_pair("Complement", (int) COMPLEMENT));
    methods.push_back(std::make_pair("Intersection", (int) INTERSECTION));
    methods.push_back(std::make_pair("Union", (int) UNION));
    addParameter(param::ParameterFactory::declareParameterSet("operation", methods));
}

void SetOperation::allConnectorsArrived()
{
    CvMatMessage::Ptr img1 = i1_->getMessage<CvMatMessage>();

    if(img1->encoding.size() != 1) {
        throw std::runtime_error("No Single Channel!");
    }


    CvMatMessage::Ptr out(new CvMatMessage);

    int op = param<int>("operation");
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
    setSynchronizedInputs(true);

    i1_ = addInput<CvMatMessage>("Mask 1");
    i2_ = addInput<CvMatMessage>("Mask 2", true);
    out_ = addOutput<CvMatMessage>("Combined");
}
