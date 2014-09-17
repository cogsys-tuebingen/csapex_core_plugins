/// HEADER
#include "file_importer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/manager/message_provider_manager.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

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

void FileImporter::setupParameters()
{
    std::string filter = std::string("Supported files (") + MessageProviderManager::instance().supportedTypes() + ");;All files (*.*)";
    addParameter(param::ParameterFactory::declareFileInputPath("path", "", filter), boost::bind(&FileImporter::import, this));

    param::Parameter::Ptr immediate = param::ParameterFactory::declareBool("playback/immediate", false);
    addParameter(immediate, boost::bind(&FileImporter::changeMode, this));
}

FileImporter::~FileImporter()
{
}

void FileImporter::changeMode()
{
    if(readParameter<bool>("playback/immediate")) {
        getNodeWorker()->setTickFrequency(1000.0);
    } else {
        getNodeWorker()->setTickFrequency(readParameter<double>("playback/frequency"));
    }
}

void FileImporter::tick()
{
    if(provider_.get() && provider_->hasNext()) {
        Message::Ptr msg = provider_->next();
        if(msg.get()) {
            output_->setType(provider_->getType());
            output_->setLabel(provider_->getType()->name());
            output_->publish(msg);
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
    provider_->load(path.toStdString());

    setTemporaryParameters(provider_->getParameters(), boost::bind(&FileImporter::updateProvider, this));

    return provider_.get();
}

void FileImporter::setup()
{
    output_ = modifier_->addOutput<connection_types::AnyMessage>("Unknown");

    param::Parameter::Ptr immediate = getParameter("playback/immediate");

    boost::function<void(param::Parameter*)> setf = boost::bind(&NodeWorker::setTickFrequency, getNodeWorker(), boost::bind(&param::Parameter::as<double>, _1));
    boost::function<bool()> conditionf = (!boost::bind(&param::Parameter::as<bool>, immediate.get()));
    addConditionalParameter(param::ParameterFactory::declareRange("playback/frequency", 1.0, 256.0, 30.0, 0.5), conditionf, setf);
}

void FileImporter::process()
{

}

void FileImporter::updateProvider()
{
    provider_->parameterChanged();
}

void FileImporter::import()
{
    doImport(QString::fromStdString(readParameter<std::string>("path")));
}
