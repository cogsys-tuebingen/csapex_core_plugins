/// HEADER
#include "confusion_matrix_display.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/factory/message_factory.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/profiling/timer.h>

#include <csapex/msg/any_message.h>

/// SYSTEM
#include <fstream>
#include <QDir>

CSAPEX_REGISTER_CLASS(csapex::ConfusionMatrixDisplay, csapex::Node)

using namespace csapex;

ConfusionMatrixDisplay::ConfusionMatrixDisplay()
    : connector_(nullptr)
{
}

void ConfusionMatrixDisplay::setupParameters(Parameterizable &params)
{
    params.addParameter(csapex::param::ParameterFactory::declareText(
                                "filename",
                                csapex::param::ParameterDescription("Name of csv file"),
                                "conf_mat"),
                            filename_);

    params.addParameter(csapex::param::ParameterFactory::declareDirectoryOutputPath(
                                "path",
                                csapex::param::ParameterDescription("Directory to write file to"),
                                "",
                                ""),
                            path_);

    params.addParameter(param::ParameterFactory::declareTrigger("save csv file"),
                            [this](param::Parameter*) {
        save();
    });

}

void ConfusionMatrixDisplay::save()
{
    int suffix = 0;
    while(true) {
        std::stringstream file_s;

        file_s << path_ << "/" << filename_ << "_" << suffix << ".csv";

        std::string file = file_s.str();

        if(!QFile(QString::fromStdString(file)).exists()) {
            exportCsv(file);
            break;
        }
        else {
        ++suffix;
        }
    }
}

void ConfusionMatrixDisplay::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
}
void ConfusionMatrixDisplay::process()
{
    connection_types::ConfusionMatrixMessage::ConstPtr msg = msg::getMessage<connection_types::ConfusionMatrixMessage>(connector_);

    {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        confusion_ = msg->confusion;
    }

    display_request();
}

ConfusionMatrix ConfusionMatrixDisplay::getConfusionMatrix() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return confusion_;
}

void ConfusionMatrixDisplay::exportCsv(const std::string &file)
{
    if(!file.empty()) {
        ConfusionMatrix cm = getConfusionMatrix();

        std::ofstream of(file);

        // row = prediction, col = actual
        std::vector<int> classes = cm.classes;
        std::sort(classes.begin(), classes.end());

        of << "classes";
        for(int cl : classes) {
            of << ',' << cl;
        }
        of << '\n';

        for(int prediction : classes) {
            of << prediction;
            for(int actual : classes) {
                of << ',' << cm.histogram.at(std::make_pair(actual, prediction));
            }
            of << '\n';
        }

        of.close();
    }
}

