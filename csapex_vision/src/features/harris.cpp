/// HEADER
#include "harris.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::CornerHarris, csapex::Node)

CornerHarris::CornerHarris() : k_(100.0), k_size_(1), block_size_(3), border_type_(cv::BORDER_DEFAULT)
{
}

void CornerHarris::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);

    if (!in->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds));

    cv::cornerHarris(in->value, out->value, block_size_, k_size_, k_, border_type_);

    msg::publish(output_, out);
}

void CornerHarris::setup(NodeModifier& node_modifier)
{
    CornerLineDetection::setup(node_modifier);
    update();
}

void CornerHarris::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("k", 1.0, 400.0, 100.0, 1.0), std::bind(&CornerHarris::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("block size", 3, 31, 3, 2), std::bind(&CornerHarris::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("k size", 1, 31, 1, 2), std::bind(&CornerHarris::update, this));

    std::map<std::string, int> types = {
        { "BORDER_DEFAULT", (int)cv::BORDER_DEFAULT },       { "BORDER_CONSTANT", (int)cv::BORDER_CONSTANT },       { "BORDER_REFLECT", (int)cv::BORDER_REFLECT },
        { "BORDER_REFLECT101", (int)cv::BORDER_REFLECT101 }, { "BORDER_REFLECT_101", (int)cv::BORDER_REFLECT_101 }, { "BORDER_REPLICATE", (int)cv::BORDER_REPLICATE }
    };

    parameters.addParameter(csapex::param::factory::declareParameterSet<int>("border type", types, (int)cv::BORDER_DEFAULT), std::bind(&CornerHarris::update, this));
}

void CornerHarris::update()
{
    k_ = readParameter<double>("k");
    k_size_ = readParameter<int>("k size");
    block_size_ = readParameter<int>("block size");
    border_type_ = readParameter<int>("border type");
}
