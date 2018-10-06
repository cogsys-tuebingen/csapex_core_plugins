
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <QDir>
#include <fstream>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class ExportCvMatMessageAsCsv : public Node
{
public:
    ExportCvMatMessageAsCsv()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<CvMatMessage>("Input");
    }

    void setupParameters(Parameterizable& params) override
    {
        params.addParameter(param::factory::declareText("delimiter", csapex::param::ParameterDescription("Delimiter of csv file"), ";"), delimiter_);

        params.addParameter(param::factory::declareText("filename", csapex::param::ParameterDescription("Name of csv file"), "image"), filename_);

        params.addParameter(param::factory::declareDirectoryOutputPath("path", csapex::param::ParameterDescription("Directory to write file to"), "", ""), path_);

        params.addParameter(param::factory::declareBool("save_one", true), save_one_);

        params.addConditionalParameter(param::factory::declareTrigger("save csv file"), save_one_, [this](param::Parameter*) { save(); });
    }

    void process() override
    {
        CvMatMessage::ConstPtr mat_msg = msg::getMessage<CvMatMessage>(in_);

        mat_ = mat_msg->value;
        if (!save_one_) {
            save();
        }
    }

    void save()
    {
        int suffix = 0;
        while (true) {
            std::stringstream file_s;

            file_s << path_ << "/" << filename_ << "_" << suffix << ".csv";

            std::string file = file_s.str();

            if (!QFile(QString::fromStdString(file)).exists()) {
                switch (mat_.depth()) {
                    case CV_8U:
                        exportCsv<uint8_t>(file, delimiter_);
                        break;
                    case CV_8S:
                        exportCsv<int8_t>(file, delimiter_);
                        break;
                    case CV_16U:
                        exportCsv<uint16_t>(file, delimiter_);
                        break;
                    case CV_16S:
                        exportCsv<int16_t>(file, delimiter_);
                        break;
                    case CV_32S:
                        exportCsv<int32_t>(file, delimiter_);
                        break;
                    case CV_32F:
                        exportCsv<float>(file, delimiter_);
                        break;
                    case CV_64F:
                        exportCsv<double>(file, delimiter_);
                        break;
                    default:
                        break;
                }
                break;
            } else {
                ++suffix;
            }
        }
    }

    template <typename T>
    void exportCsv(const std::string& file, std::string delimiter = ";")
    {
        if (!file.empty()) {
            std::ofstream of(file);
            for (int i = 0; i < mat_.rows; ++i) {
                for (int j = 0; j < mat_.cols; ++j) {
                    of << mat_.at<T>(i, j);
                    if (j != mat_.cols - 1) {
                        of << delimiter;
                    }
                }
                of << std::endl;
            }
            of.close();
        }
    }

private:
    bool save_one_;
    Input* in_;
    Output* out_;
    cv::Mat mat_;
    std::string delimiter_;
    std::string filename_;
    std::string path_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ExportCvMatMessageAsCsv, csapex::Node)
