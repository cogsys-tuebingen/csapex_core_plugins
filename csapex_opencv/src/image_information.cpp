
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class ImageInformation : public Node
{
public:
    ImageInformation()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<CvMatMessage>("Image");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareOutputText("Information"));
    }

    void process() override
    {
        CvMatMessage::ConstPtr image = msg::getMessage<CvMatMessage>(in_);
        cv::Mat& mat = image->value;

        std::stringstream info;

        info << "Width: " << mat.cols << '\n';
        info << "Height: " << mat.rows << '\n';
        info << "Type: " << getImageType(mat.type()) << '\n';
        info << "Encoding" << image->getEncoding().toString();

        setParameter("Information", info.str());
    }

    std::string getImageType(int number)
    {
        // taken from https://stackoverflow.com/questions/12335663/getting-enum-names-e-g-cv-32fc1-of-opencv-image-types
        // find type
        int imgTypeInt = number%8;
        std::string imgTypeString;

        switch (imgTypeInt)
        {
            case 0:
                imgTypeString = "8U";
                break;
            case 1:
                imgTypeString = "8S";
                break;
            case 2:
                imgTypeString = "16U";
                break;
            case 3:
                imgTypeString = "16S";
                break;
            case 4:
                imgTypeString = "32S";
                break;
            case 5:
                imgTypeString = "32F";
                break;
            case 6:
                imgTypeString = "64F";
                break;
            default:
                break;
        }

        // find channel
        int channel = (number/8) + 1;

        std::stringstream type;
        type<<"CV_"<<imgTypeString<<"C"<<channel;

        return type.str();
    }


private:
    Input* in_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::ImageInformation, csapex::Node)

