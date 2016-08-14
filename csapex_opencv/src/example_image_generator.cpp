/// PROJECT
#include <csapex/model/tickable_node.h>
#include <csapex/msg/io.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/view/utility/QtCvImageConverter.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <QImage>

namespace csapex
{

class ExampleImageGenerator : public TickableNode
{
public:
    ExampleImageGenerator()
    {

    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        output_ = node_modifier.addOutput<connection_types::CvMatMessage>("Image");

        image = QtCvImageConverter::Converter<QImage>::QImage2Mat(QImage(":/lena.png"));
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {

    }

    virtual void tick() override
    {
        connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::bgr, 0));
        msg->value = image.clone();

        if(msg->value.channels() == 1)
            msg->setEncoding(enc::mono);

        msg::publish(output_, msg);
    }

    virtual bool canTick() override
    {
        return true;
    }

private:
    Output* output_;

    cv::Mat image;
};

}


CSAPEX_REGISTER_CLASS(csapex::ExampleImageGenerator, csapex::Node)
