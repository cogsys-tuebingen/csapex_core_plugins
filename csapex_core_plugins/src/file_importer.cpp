/// HEADER
#include "file_importer.h"

/// PROJECT
#include <csapex/manager/message_provider_manager.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/signal/trigger.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/timer.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/range_parameter.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/lambda/lambda.hpp>
#include <QDir>
#include <QUrl>

CSAPEX_REGISTER_CLASS(csapex::FileImporter, csapex::Node)


using namespace csapex;
using namespace connection_types;


FileImporter::FileImporter()
    :  directory_import_(false), last_directory_index_(-1)
{
}

FileImporter::~FileImporter()
{
}


void FileImporter::setupParameters(Parameterizable& parameters)
{
    param::Parameter::Ptr directory = param::ParameterFactory::declareBool("import directory", false);
    parameters.addParameter(directory);

    std::function<bool()> cond_file = [directory]() { return !directory->as<bool>(); };
    std::function<bool()> cond_dir = [directory]() { return directory->as<bool>(); };

    parameters.addConditionalParameter(param::ParameterFactory::declareBool("recursive import", false), cond_dir);

    std::string filter = std::string("Supported files (") + MessageProviderManager::instance().supportedTypes() + ");;All files (*.*)";
    parameters.addConditionalParameter(param::ParameterFactory::declareFileInputPath("path", "", filter), cond_file, std::bind(&FileImporter::import, this));

    parameters.addConditionalParameter(param::ParameterFactory::declareDirectoryInputPath("directory", ""), cond_dir, std::bind(&FileImporter::import, this));
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<int>("directory/current", 0, 1, 0, 1), cond_dir, std::bind(&FileImporter::changeDirIndex, this));

    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/play", true), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/loop", true), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/latch", false), cond_dir);

    param::Parameter::Ptr immediate = param::ParameterFactory::declareBool("playback/immediate", false);
    parameters.addParameter(immediate, std::bind(&FileImporter::changeMode, this));

    std::function<void(param::Parameter*)> setf = std::bind(&NodeModifier::setTickFrequency, modifier_, std::bind(&param::Parameter::as<double>, std::placeholders::_1));
    std::function<bool()> conditionf = [immediate]() { return !immediate->as<bool>(); };
    addConditionalParameter(param::ParameterFactory::declareRange("playback/frequency", 1.0, 256.0, 30.0, 0.5), conditionf, setf);
}

void FileImporter::setup(NodeModifier& node_modifier)
{
    outputs_.push_back(node_modifier.addOutput<connection_types::AnyMessage>("Unknown"));

    begin_ = node_modifier.addTrigger("begin");
    end_ = node_modifier.addTrigger("end");
}

void FileImporter::changeMode()
{
    if(readParameter<bool>("playback/immediate")) {
        modifier_->setTickFrequency(-1.0);
    } else {
        modifier_->setTickFrequency(readParameter<double>("playback/frequency"));
    }
}

void FileImporter::process()
{
    tick();
}

bool FileImporter::canTick()
{
    if(!modifier_->isSource()) {
        return false;
    }
    if(directory_import_) {
        return (readParameter<int>("directory/current") < (int) dir_files_.size()) ||
                readParameter<bool>("directory/loop")  ||
                readParameter<bool>("directory/latch");
    } else {
        return provider_ != nullptr;
    }
}

void FileImporter::tick()
{
    if(provider_ && provider_->hasNext()) {
        for(std::size_t slot = 0, total = provider_->slotCount(); slot < total; ++slot) {
            INTERLUDE_STREAM("slot " << slot);

            Output* output = outputs_[slot];

            if(msg::isConnected(output)) {
                Message::Ptr msg = provider_->next(slot);
                if(msg) {
                    msg::publish(output, msg);
                }
            }
        }
    } else if(directory_import_) {
        bool play = readParameter<bool>("directory/play");
        bool latch = readParameter<bool>("directory/latch");
        bool loop = readParameter<bool>("directory/loop");

        int current = readParameter<int>("directory/current");
        bool index_changed = current != last_directory_index_;

        if(!play && !latch && !index_changed) {
            return;
        }
        INTERLUDE("directory");

        int files = dir_files_.size();

        if(current >= files) {
            end_->trigger();
            if(loop) {
                current = 0;
            } else if(latch && files > 0) {
                current = files - 1;
            }
        }

        if(current < files) {
            if(current == 0) {
                begin_->trigger();
            }

            if(doImport(QString::fromStdString(dir_files_[current]))) {
                tick();
            }

            if(play) {
                ++current;
            }
        }
        setParameter("directory/current", current);
        last_directory_index_ = current;
    }
}

void FileImporter::doImportDir(const QString &dir_string)
{

    dir_files_.clear();

    bool recursive = readParameter<bool>("recursive import");

    std::function<void(const boost::filesystem::path&)> crawl_dir = [&](const boost::filesystem::path& directory) {
        boost::filesystem::directory_iterator dir(directory);
        boost::filesystem::directory_iterator end;
        for(; dir != end; ++dir) {
            boost::filesystem::path path = dir->path();

            if(boost::filesystem::is_directory(path)) {
                if(recursive) {
                    crawl_dir(path);
                }
            } else {
                dir_files_.push_back(path.string());
            }

        }
    };
    boost::filesystem::path directory(dir_string.toStdString());
    crawl_dir(directory);

    std::sort(dir_files_.begin(), dir_files_.end());

    param::RangeParameter::Ptr current = getParameter<param::RangeParameter>("directory/current");
    current->set(0);
    current->setMax<int>(dir_files_.size());
}

bool FileImporter::doImport(const QString& file_path)
{
    INTERLUDE("doImport");
    if(file_path.isEmpty()) {
        modifier_->setWarning("no file selected");
        return false;
    }
    bool latch = readParameter<bool>("directory/latch");
    if((file_path == file_ && !latch) && provider_) {
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
            modifier_->setError(std::string("the file ") + file_path.toStdString() + " couldn't be opened");
            return false;
        }
    }

    file_ = file_path;
    modifier_->setNoError();

    try {
        {
            INTERLUDE("createMessageProvider");
            provider_ = MessageProviderManager::createMessageProvider(path.toStdString());
        }
        provider_->slot_count_changed.connect(std::bind(&FileImporter::updateOutputs, this));

        if(!directory_import_) {
            provider_->begin.connect(std::bind(&Trigger::trigger, begin_));
            provider_->no_more_messages.connect(std::bind(&Trigger::trigger, end_));
        }

        {
            INTERLUDE("load");
            provider_->load(path.toStdString());
        }

        if(!directory_import_) {
            setTemporaryParameters(provider_->getParameters(), std::bind(&FileImporter::updateProvider, this));
        }
        return provider_.get();

    } catch(const std::exception& e) {
        modifier_->setNoError();
        throw std::runtime_error(std::string("cannot load file ") + file_.toStdString() + ": " + e.what());
    }

    return nullptr;
}

void FileImporter::updateProvider()
{
    if(provider_) {
        provider_->parameterChanged();
    }
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
            if(msg::isConnected(output)) {
                del = false;
            }

            if(del) {
                modifier_->removeOutput(msg::getUUID(output));
                outputs_.erase(outputs_.begin() + i);
            } else {
                msg::disable(output);
            }
        }
    }

    for(std::size_t i = 0; i < slot_count; ++i) {
        Output* out = outputs_[i];
        msg::setLabel(out, provider_->getLabel(i));
    }
}

void FileImporter::changeDirIndex()
{
    //    int set_to = readParameter<int>("directory/current");
    //    setParameter("directory/current", set_to);
}

void FileImporter::import()
{
    directory_import_ = readParameter<bool>("import directory");
    provider_.reset();

    if(directory_import_) {
        doImportDir(QString::fromStdString(readParameter<std::string>("directory")));
        removeTemporaryParameters();
    } else {
        doImport(QString::fromStdString(readParameter<std::string>("path")));
    }
}
