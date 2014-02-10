/// HEADER
#include "harris.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <boost/assign/std.hpp>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::CornerHarris, csapex::Node)

CornerHarris::CornerHarris() :
    k_(100.0),
    k_size_(1),
    block_size_(3),
    border_type_(cv::BORDER_DEFAULT)
{
    addParameter(param::ParameterFactory::declareRange("k", 1.0, 400.0, 100.0, 1.0),
                 boost::bind(&CornerHarris::update, this));
    addParameter(param::ParameterFactory::declareRange("block size", 3, 31, 3, 2),
                 boost::bind(&CornerHarris::update, this));
    addParameter(param::ParameterFactory::declareRange("k size", 1, 31, 1, 2),
                 boost::bind(&CornerHarris::update, this));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("BORDER_DEFAULT", (int) cv::BORDER_DEFAULT)
            ("BORDER_CONSTANT", (int) cv::BORDER_CONSTANT)
            ("BORDER_REFLECT", (int) cv::BORDER_REFLECT)
            ("BORDER_REFLECT101", (int) cv::BORDER_REFLECT101)
            ("BORDER_REFLECT_101", (int) cv::BORDER_REFLECT_101)
            ("BORDER_REPLICATE", (int) cv::BORDER_REPLICATE);
    addParameter(param::ParameterFactory::declareParameterSet<int>("border type", types),
                 boost::bind(&CornerHarris::update, this));

}

void CornerHarris::allConnectorsArrived()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();

    if(in->getEncoding() != enc::mono) {
        throw std::runtime_error("image must be grayscale.");
    }

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono));

    cv::cornerHarris(in->value, out->value, block_size_, k_size_, k_, border_type_);

    output_->publish(out);
}

void CornerHarris::setup()
{
    CornerDetection::setup();
    update();
}

void CornerHarris::update()
{
    k_ = param<double>("k");
    k_size_ = param<int>("k size");
    block_size_ = param<int>("block size");
    border_type_ = param<int>("border type");
}
