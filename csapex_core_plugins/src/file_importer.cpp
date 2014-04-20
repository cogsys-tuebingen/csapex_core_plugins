/// HEADER
#include "file_importer.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/manager/message_provider_manager.h>
#include <utils_param/parameter_factory.h>

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

CSAPEX_REGISTER_CLASS(csapex::FileImporter, csapex::Node)

using namespace csapex;
using namespace connection_types;


FileImporter::FileImporter()
{
    addTag(Tag::get("General"));
    addTag(Tag::get("Input"));

    std::string filter = std::string("Supported files (") + MessageProviderManager::instance().supportedTypes() + ");;All files (*.*)";
    addParameter(param::ParameterFactory::declareFileInputPath("path", "", filter), boost::bind(&FileImporter::import, this));
}

FileImporter::~FileImporter()
{
}


void FileImporter::tick()
{
    if(provider_.get()) {
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
