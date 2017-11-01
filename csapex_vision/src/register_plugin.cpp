/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex_vision/ros_converters.h>

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>
#include <csapex/factory/generic_node_factory.hpp>
#include <csapex/factory/node_factory_local.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QMetaType>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <compressed_image_transport/compressed_subscriber.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterPlugin, csapex::CorePlugin)

using namespace csapex;

RegisterPlugin::RegisterPlugin()
{
}

void testWrap(const connection_types::CvMatMessage& input, int flipcode,
              connection_types::CvMatMessage& output)
{
    cv::flip(input.value, output.value, flipcode);
}

void testWrapDirect(const cv::Mat& input, int flipcode,
                    cv::Mat& output)
{
    cv::flip(input, output, flipcode);
}

struct ParameterInfoTestWrap
{
    static std::string getName(int index) {
        switch(index) {
        case 0:
            return "Image";
        case 1:
            return "flip code";
        case 2:
            return "Flipped Image";
        default:
            return "";
        }
    }
    template <typename P>
    static csapex::param::ParameterPtr declareParameter(int index) {
        switch(index) {
        case 1:
            return csapex::param::ParameterFactory::declareRange<P>(getName(index), -1, 3, 0, 1);
        default:
            return nullptr;
        }
    }
};

void RegisterPlugin::init(CsApexCore& core)
{
    RosMessageConversion::registerConversion<sensor_msgs::Image, connection_types::CvMatMessage, Image2CvMat>();
    RosMessageConversion::registerConversion<sensor_msgs::CompressedImage, connection_types::CvMatMessage, CompressedImage2CvMat>();
    RosMessageConversion::registerConversion<sensor_msgs::RegionOfInterest, connection_types::RoiMessage, ConvertROI>();

    auto cWrap = GenericNodeFactory::createConstructorFromFunction<ParameterInfoTestWrap>
            (testWrap, "TestWrap");
    cWrap->setDescription("Test direct wrapping.").setIcon(":/combiner.png").setTags({"Wrap", "Test", "Flip"});
    core.getNodeFactory()->registerNodeType(cWrap);

    auto cWrapDirect = GenericNodeFactory::createConstructorFromFunction<ParameterInfoTestWrap>
            (testWrapDirect, "TestWrapDirect");
    cWrapDirect->setDescription( "Test directly wrapping a non-apex function.").setIcon(":/combiner.png").setTags({"Wrap", "Test", "Flip"});
    core.getNodeFactory()->registerNodeType(cWrapDirect);

    auto cWrapDirectNoInfo = GenericNodeFactory::createConstructorFromFunction
            (testWrapDirect, "TestWrapDirectNoInfo");
    cWrapDirectNoInfo->setDescription("Test directly wrapping a non-apex function with default info.");
    core.getNodeFactory()->registerNodeType(cWrapDirectNoInfo);

}
