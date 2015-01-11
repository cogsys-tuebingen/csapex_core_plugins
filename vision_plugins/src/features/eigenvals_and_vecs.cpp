/// HEADER
#include "eigenvals_and_vecs.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <boost/assign/std.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::EigenValsAndVecs, csapex::Node)

EigenValsAndVecs::EigenValsAndVecs() :
    k_size_(1),
    block_size_(3),
    border_type_(cv::BORDER_DEFAULT)
{
}

void EigenValsAndVecs::process()
{
    CvMatMessage::ConstPtr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp));

    cv::Mat tmp;
    if(in->value.type() != CV_8UC1) {
        cv::cvtColor(in->value, tmp, CV_BGR2GRAY);
    } else {
        tmp = in->value;
    }
    if(eigen_type_ == MIN_EIGEN_VAL)
       cv::cornerMinEigenVal(tmp, tmp, block_size_, k_size_, border_type_);
    else
       cv::cornerEigenValsAndVecs(tmp, tmp, block_size_, k_size_, border_type_);

    out->value = tmp.clone();
    output_->publish(out);
}

void EigenValsAndVecs::setup()
{
    CornerLineDetection::setup();
    update();
}

void EigenValsAndVecs::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("k", 1.0, 400.0, 100.0, 1.0),
                 boost::bind(&EigenValsAndVecs::update, this));
    addParameter(param::ParameterFactory::declareRange("block size", 3, 31, 3, 2),
                 boost::bind(&EigenValsAndVecs::update, this));
    addParameter(param::ParameterFactory::declareRange("k size", 1, 31, 1, 2),
                 boost::bind(&EigenValsAndVecs::update, this));

    std::map<std::string, int> border_types = boost::assign::map_list_of
            ("BORDER_DEFAULT", (int) cv::BORDER_DEFAULT)
            ("BORDER_CONSTANT", (int) cv::BORDER_CONSTANT)
            ("BORDER_REFLECT", (int) cv::BORDER_REFLECT)
            ("BORDER_REFLECT101", (int) cv::BORDER_REFLECT101)
            ("BORDER_REFLECT_101", (int) cv::BORDER_REFLECT_101)
            ("BORDER_REPLICATE", (int) cv::BORDER_REPLICATE);

    addParameter(param::ParameterFactory::declareParameterSet<int>("border type", border_types, (int) cv::BORDER_DEFAULT),
                 boost::bind(&EigenValsAndVecs::update, this));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("MIN_EIGEN_VAL", (int) MIN_EIGEN_VAL)
            ("EIGEN_VALS_AND_VECS", (int) EIGEN_VALS_AND_VECS);

    addParameter(param::ParameterFactory::declareParameterSet<int>("eigen type", types, (int) MIN_EIGEN_VAL),
                 boost::bind(&EigenValsAndVecs::update, this));
}

void EigenValsAndVecs::update()
{
    k_size_ = readParameter<int>("k size");
    block_size_ = readParameter<int>("block size");
    border_type_ = readParameter<int>("border type");
    eigen_type_  = (EigenType) readParameter<int>("eigen type");
}
