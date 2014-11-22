/// HEADER
#include "file_importer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/manager/message_provider_manager.h>
#include <csapex/msg/message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/signal/trigger.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QLabel>
#include <QFileDialog>
#include <QTimer>
#include <QtConcurrentRun>
#include <QCheckBox>
#include <QDirIterator>
#include <QUrl>
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::FileImporter, csapex::Node)

using namespace csapex;
using namespace connection_types;


FileImporter::FileImporter()
{
}

FileImporter::~FileImporter()
{
}


void FileImporter::setupParameters()
{
    std::string filter = std::string("Supported files (") + MessageProviderManager::instance().supportedTypes() + ");;All files (*.*)";
    addParameter(param::ParameterFactory::declareFileInputPath("path", "", filter), boost::bind(&FileImporter::import, this));

    param::Parameter::Ptr immediate = param::ParameterFactory::declareBool("playback/immediate", false);
    addParameter(immediate, boost::bind(&FileImporter::changeMode, this));
}

void FileImporter::setup()
{
    outputs_.push_back(modifier_->addOutput<connection_types::AnyMessage>("Unknown"));

    param::Parameter::Ptr immediate = getParameter("playback/immediate");

    boost::function<void(param::Parameter*)> setf = boost::bind(&NodeWorker::setTickFrequency, getNodeWorker(), boost::bind(&param::Parameter::as<double>, _1));
    boost::function<bool()> conditionf = (!boost::bind(&param::Parameter::as<bool>, immediate.get()));
    addConditionalParameter(param::ParameterFactory::declareRange("playback/frequency", 1.0, 256.0, 30.0, 0.5), conditionf, setf);

    begin_ = modifier_->addTrigger("begin");
    end_ = modifier_->addTrigger("end");
}

void FileImporter::changeMode()
{
    if(readParameter<bool>("playback/immediate")) {
        getNodeWorker()->setTickFrequency(1000.0);
    } else {
        getNodeWorker()->setTickFrequency(readParameter<double>("playback/frequency"));
    }
}

void FileImporter::process()
{

}

void FileImporter::tick()
{
    if(provider_) {
        if(provider_->hasNext()) {
            for(std::size_t slot = 0, total = provider_->slotCount(); slot < total; ++slot) {
                Output* output = outputs_[slot];

                if(output->isConnected()) {
                    Message::Ptr msg = provider_->next(slot);
                    if(msg) {
                        output->publish(msg);
                    }
                }
            }
        }
    }
}

bool FileImporter::doImport(const QString& _path)
{
    if(_path == file_) {
        return false;
    }

    QString path;
    QFile file(path);
    if(file.exists()) {
        path = _path;
    } else {
        QFile urlfile(QUrl(_path).toLocalFile());

        if(urlfile.exists()) {
            path = urlfile.fileName();
        } else {
            setError(true, std::string("the file ") + _path.toStdString() + " couldn't be opened");
            return false;
        }
    }

    file_ = _path;
    setError(false);

    provider_ = MessageProviderManager::createMessageProvider(path.toStdString());
    provider_->slot_count_changed.connect(boost::bind(&FileImporter::updateOutputs, this));

    provider_->begin.connect(boost::bind(&Trigger::trigger, begin_));
    provider_->no_more_messages.connect(boost::bind(&Trigger::trigger, end_));

    provider_->load(path.toStdString());

    setTemporaryParameters(provider_->getParameters(), boost::bind(&FileImporter::updateProvider, this));

    return provider_.get();
}

void FileImporter::updateProvider()
{
    provider_->parameterChanged();
}

void FileImporter::updateOutputs()
{
    std::size_t slot_count = provider_->slotCount();

    std::size_t output_count = outputs_.size();

    if(slot_count > output_count) {
        for(std::size_t i = output_count ; i < slot_count ; ++i) {
            outputs_.push_back(modifier_->addOutput<AnyMessage>("unknown"));
        }
    } else {
        bool del = true;
        for(int i = output_count-1 ; i >= (int) slot_count; --i) {
            Output* output = outputs_[i];
            if(output->isConnected()) {
                del = false;
            }

            if(del) {
                getNodeWorker()->removeOutput(output->getUUID());
                outputs_.erase(outputs_.begin() + i);
            } else {
                output->disable();
            }
        }
    }

    for(std::size_t i = 0; i < slot_count; ++i) {
        Output* out = outputs_[i];
        out->setLabel(provider_->getLabel(i));
    }
}

void FileImporter::import()
{
    doImport(QString::fromStdString(readParameter<std::string>("path")));
}
