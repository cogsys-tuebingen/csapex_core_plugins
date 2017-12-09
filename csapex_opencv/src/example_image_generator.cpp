/// PROJECT
#include <csapex/model/node.h>
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

class CSAPEX_EXPORT_PLUGIN ExampleImageGenerator : public Node
{
public:
    enum class Images
    {
        LENA,
        NUMBER,
        CAT
    };

    ExampleImageGenerator()
        : seq_(0)
    {

    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        output_ = node_modifier.addOutput<connection_types::CvMatMessage>("Image");

        lena_ = QtCvImageConverter::Converter<QImage>::QImage2Mat(QImage(":/lena.png"));
        cat_ = QtCvImageConverter::Converter<QImage>::QImage2Mat(QImage(":/cat.jpg"));
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        std::map<std::string, int> set {
            {"LENA", (int) Images::LENA},
            {"NUMBER", (int) Images::NUMBER},
            {"CAT", (int) Images::CAT}
        };
        parameters.addParameter(param::ParameterFactory::declareParameterSet("image", set, (int) Images::LENA),
                                image_type_);
    }

    virtual void process() override
    {
        connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::bgr, "camera", 0));

        switch(image_type_) {
        case Images::LENA:
            msg->value = lena_.clone();
            break;
        case Images::CAT:
            msg->value = cat_.clone();
            break;
        case Images::NUMBER:

            msg->value = cv::Mat(400, 400, CV_8UC3);

            std::stringstream txt;
            txt << seq_;
            cv::rectangle(msg->value, cv::Rect(0,0, msg->value.cols, msg->value.rows), cv::Scalar::all(0), CV_FILLED);
            cv::putText(msg->value, txt.str(), cv::Point(20, 200), CV_FONT_HERSHEY_PLAIN, 5.0, cv::Scalar::all(255), 2, CV_AA);

            msg::publish(output_, msg);
            break;
        }


        if(msg->value.channels() == 1)
            msg->setEncoding(enc::mono);

        ++seq_;

        msg::publish(output_, msg);
    }

private:
    Output* output_;

    cv::Mat lena_;
    cv::Mat cat_;

    Images image_type_;

    int seq_;
};

}


CSAPEX_REGISTER_CLASS(csapex::ExampleImageGenerator, csapex::Node)
