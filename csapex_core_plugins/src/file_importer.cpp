/// HEADER
#include "file_importer.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/manager/message_provider_manager.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QLabel>
#include <QFileDialog>
#include <QTimer>
#include <QtConcurrentRun>
#include <QCheckBox>
#include <QDirIterator>
#include <csapex/utility/register_apex_plugin.h>
#include <QUrl>
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::FileImporter, csapex::Node)

using namespace csapex;
using namespace connection_types;


FileImporter::FileImporter()
{
    addTag(Tag::get("General"));
    addTag(Tag::get("Input"));

    std::string filter = std::string("Supported files (") + MessageProviderManager::instance().supportedTypes() + ");;All files (*.*)";
    addParameter(param::ParameterFactory::declareFileInputPath("path", "", filter), boost::bind(&FileImporter::import, this));

    param::Parameter::Ptr immediate = param::ParameterFactory::declareBool("playback/immediate", false);
    addParameter(immediate, boost::bind(&FileImporter::changeMode, this));

    boost::function<void(param::Parameter*)> setf = boost::bind(&NodeWorker::setTickFrequency, getNodeWorker(), boost::bind(&param::Parameter::as<double>, _1));
    boost::function<bool()> conditionf = (!boost::bind(&param::Parameter::as<bool>, immediate.get()));
    addConditionalParameter(param::ParameterFactory::declareRange("playback/frequency", 1.0, 256.0, 30.0, 0.5), conditionf, setf);
}

FileImporter::~FileImporter()
{
}

void FileImporter::changeMode()
{
    if(param<bool>("playback/immediate")) {
        getNodeWorker()->setTickFrequency(1000.0);
    } else {
        getNodeWorker()->setTickFrequency(param<double>("playback/frequency"));
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

    setError(false);

    provider_ = MessageProviderManager::createMessageProvider(path.toStdString());
    provider_->load(path.toStdString());


    removeTemporaryParameters();

    std::vector<param::Parameter::Ptr> params = provider_->getParameters();
    Q_FOREACH(param::Parameter::Ptr param, params) {
        addTemporaryParameter(param);
    }

    return provider_.get();
}

QIcon FileImporter::getIcon() const
{
    return QIcon(":/folder_picture.png");
}

void FileImporter::setup()
{
    setSynchronizedInputs(true);

    output_ = addOutput<connection_types::AnyMessage>("Unknown");
}

void FileImporter::process()
{

}

void FileImporter::import()
{
    doImport(QString::fromStdString(param<std::string>("path")));
}
