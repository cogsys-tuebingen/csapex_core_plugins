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
#include <boost/filesystem.hpp>
#include <QLabel>
#include <QFileDialog>
#include <QTimer>
#include <QtConcurrentRun>
#include <QCheckBox>
#include <QUrl>
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::FileImporter, csapex::Node)

using namespace csapex;
using namespace connection_types;


FileImporter::FileImporter()
    : directory_import_(false)
{
}

FileImporter::~FileImporter()
{
}


void FileImporter::setupParameters()
{
    param::Parameter::Ptr directory = param::ParameterFactory::declareBool("import directory", false);
    addParameter(directory);

    boost::function<bool()> cond_file = (!boost::bind(&param::Parameter::as<bool>, directory.get()));
    boost::function<bool()> cond_dir = (boost::bind(&param::Parameter::as<bool>, directory.get()));


    std::string filter = std::string("Supported files (") + MessageProviderManager::instance().supportedTypes() + ");;All files (*.*)";
    addConditionalParameter(param::ParameterFactory::declareFileInputPath("path", "", filter), cond_file, boost::bind(&FileImporter::import, this));

    addConditionalParameter(param::ParameterFactory::declareDirectoryInputPath("directory", ""), cond_dir, boost::bind(&FileImporter::import, this));
    addConditionalParameter(param::ParameterFactory::declareRange<int>("directory/current", 0, 1, 0, 1), cond_dir, boost::bind(&FileImporter::import, this));

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
        getNodeWorker()->setTickFrequency(-1.0);
    } else {
        getNodeWorker()->setTickFrequency(readParameter<double>("playback/frequency"));
    }
}

void FileImporter::process()
{

}

bool FileImporter::canTick()
{
    if(directory_import_) {
        return readParameter<int>("directory/current") < (int) dir_files_.size();
    } else {
        return provider_;
    }
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
        } else {
            if(directory_import_) {
                int current = readParameter<int>("directory/current");
                ++current;

                if(current >= (int) dir_files_.size()) {

                }
            }
        }
    }
}

void FileImporter::doImportDir(const QString &dir_string)
{
    boost::filesystem::path directory(dir_string.toStdString());
    boost::filesystem::directory_iterator dir(directory);
    boost::filesystem::directory_iterator end;

    dir_files_.clear();

    for(; dir != end; ++dir) {
        boost::filesystem::path path = dir->path();

        dir_files_.push_back(path.string());
    }
}

bool FileImporter::doImport(const QString& file_path)
{
    if(file_path == file_) {
        return false;
    }

    QString path;
    QFile file(path);
    if(file.exists()) {
        path = file_path;
    } else {
        QUrl url(file_path);
        QFile urlfile(file_path);

        if(urlfile.exists()) {
            path = urlfile.fileName();
        } else {
            setError(true, std::string("the file ") + file_path.toStdString() + " couldn't be opened");
            return false;
        }
    }

    file_ = file_path;
    setError(false);

    try {
        provider_ = MessageProviderManager::createMessageProvider(path.toStdString());

        provider_->slot_count_changed.connect(boost::bind(&FileImporter::updateOutputs, this));

        provider_->begin.connect(boost::bind(&Trigger::trigger, begin_));
        provider_->no_more_messages.connect(boost::bind(&Trigger::trigger, end_));

        provider_->load(path.toStdString());

        setTemporaryParameters(provider_->getParameters(), boost::bind(&FileImporter::updateProvider, this));

        return provider_.get();

    } catch(const std::exception& e) {
        setError(true);
        throw std::runtime_error(std::string("cannot load file ") + file_.toStdString() + ": " + e.what());
    }

    return NULL;
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
    directory_import_ = readParameter<bool>("import directory");

    if(directory_import_) {
        doImportDir(QString::fromStdString(readParameter<std::string>("directory")));
    } else {
        doImport(QString::fromStdString(readParameter<std::string>("path")));
    }
}
