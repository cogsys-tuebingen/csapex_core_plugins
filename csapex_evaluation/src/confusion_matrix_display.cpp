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

CSAPEX_REGISTER_CLASS(csapex::ConfusionMatrixDisplay, csapex::Node)

using namespace csapex;

ConfusionMatrixDisplay::ConfusionMatrixDisplay()
    : connector_(nullptr)
{
}

void ConfusionMatrixDisplay::setupParameters(Parameterizable &params)
{
    params.addParameter(param::ParameterFactory::declareTrigger("export as .csv file"),
                        [this](param::Parameter* p) {
        export_request();
    });
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

const ConfusionMatrix& ConfusionMatrixDisplay::getConfusionMatrix() const
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
