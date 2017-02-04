/// HEADER
#include "file_importer.h"

/// PROJECT
#include <csapex/manager/message_provider_manager.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/profiling/timer.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/msg/any_message.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/signal/slot.h>
#include <csapex/model/node_handle.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/end_of_program_message.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/lambda/lambda.hpp>
#include <QDir>
#include <QUrl>
#include <boost/regex.hpp>

CSAPEX_REGISTER_CLASS(csapex::FileImporter, csapex::Node)


using namespace csapex;
using namespace connection_types;


FileImporter::FileImporter()
    : playing_(false), abort_(false), end_triggered_(false),
      trigger_signal_end_(false), import_requested_(false),
      directory_import_(false), last_directory_index_(-1), cache_enabled_(false)
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

    parameters.addParameter(param::ParameterFactory::declareBool("split_container_messages",
                                                                 param::ParameterDescription("Specifies, wheter imported container messages should be split into their"
                                                                                             "constituent parts for export. (Only applicable to the first output slot.)"),
                                                                 false),
                            split_container_messages_);

    std::string filter = std::string("Supported files (") + MessageProviderManager::instance().supportedTypes() + ");;All files (*.*)";
    parameters.addConditionalParameter(param::ParameterFactory::declareFileInputPath("path", "", filter), cond_file, std::bind(&FileImporter::requestImport, this));

    parameters.addConditionalParameter(param::ParameterFactory::declareDirectoryInputPath("directory", ""), cond_dir, std::bind(&FileImporter::requestImport, this));
    parameters.addConditionalParameter(param::ParameterFactory::declareText("directory/current_file", ""), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareBool(
                                           "directory/sort_numerically",
                                           param::ParameterDescription(
                                               "If true, the expected format is <em>any_text</em>_<b>number</b>.<em>file_ending</em>"
                                               ),
                                           false), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareText("directory/filter",
                                                                            param::ParameterDescription("This filter is applied as a regular expression to all files in the directory."
                                                                                                        "Example usage: <b>.*\\.png</b> to select all .png files."),
                                                                            ".*"),
                                       cond_dir, [this](param::Parameter* p) {
        directory_filter_ = p->as<std::string>();
        doImportDir(QString::fromStdString(readParameter<std::string>("directory")));
    });
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/show parameters", false), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<int>("directory/current", 0, 1, 0, 1), cond_dir, std::bind(&FileImporter::changeDirIndex, this));

    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/play", true), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/loop", true), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/latch", false), cond_dir);
    parameters.addConditionalParameter(param::ParameterFactory::declareBool("directory/quit on end", false), cond_dir, quit_on_end_);

    parameters.addParameter(param::ParameterFactory::declareBool("cache", 0), [this](param::Parameter* p) {
        cache_enabled_ = p->as<bool>();
        if(!cache_enabled_) {
            cache_.clear();
        }
    });

    changeMode();
}

void FileImporter::setup(NodeModifier& node_modifier)
{
    outputs_.push_back(node_modifier.addOutput<connection_types::AnyMessage>("Unknown"));

    begin_ = node_modifier.addEvent("begin");
    end_ = node_modifier.addEvent("end");

    new_provider_ = node_modifier.addEvent("new provider");

    node_modifier.addSlot("restart", [this](){
        //DEBUGainfo << "restart" << std::endl;
        playing_ = true;
        if(directory_import_) {
            setParameter("directory/current", 0);
            int current = readParameter<int>("directory/current");
            apex_assert_hard(current == 0);
        } else {
            if(provider_) {
                provider_->restart();
            }
        }
        yield();
    });
    play_ = node_modifier.addSlot("play", [this](){
        setParameter("directory/play", true);
        playing_ = true;
        yield();
    });
    node_modifier.addSlot("stop", [this](){
        setParameter("directory/current", -1);
        setParameter("directory/play", false);
        playing_ = false;
    });
    node_modifier.addSlot("abort", [this](){
        //        playing_ = false;
        abort_ = true;
        yield();
    });
    play_->connection_added.connect([this](ConnectionPtr){
        setParameter("directory/play", false);
        playing_ = false;
    });
}

void FileImporter::changeMode()
{
}

bool FileImporter::canProcess() const
{
    if(import_requested_) {
        // only import if playing, or the play slot is unconnected
        if(playing_ || !msg::isConnected(play_)) {
            return true;
        }
    }
    if(trigger_signal_end_ || abort_) {
        return true;
    }
    if(!Node::canProcess()) {
        return false;
    }

    if(directory_import_) {
        if(play_->isConnected()) {
            bool can_tick = playing_ && readParameter<int>("directory/current") < (int) dir_files_.size();
            if(!can_tick) {
                ////DEBUGainfo << "cannot tick: " << playing_ << ", " << readParameter<int>("directory/current") << " <= " <<  (int) dir_files_.size()<< std::endl;
            }
            return can_tick;
        }
        return (readParameter<int>("directory/current") <= (int) dir_files_.size()) ||
                readParameter<bool>("directory/loop")  ||
                readParameter<bool>("directory/latch");
    } else {
        if(play_->isConnected()) {
            if(!playing_) {
                return false;
            }
        }

        if(provider_) {
            return provider_->hasNext();
        } else {
            return false;
        }
    }
}

void FileImporter::process()
{
    // check if we need to send marker messages:
    if(trigger_signal_end_) {
        signalEnd();
        return;
    }
    if(abort_) {
        abort_ = false;

        setParameter("directory/current", (int) dir_files_.size());

        end_triggered_ = false;
        signalEnd();

        return;
    }

    // check if we need to do the import
    if(import_requested_) {
        import_requested_ = false;
        import();
    }

    if(current_container_msg_) {
        if(current_container_index_ < current_container_msg_->nestedValueCount()) {
            TokenDataConstPtr msg = current_container_msg_->nestedValue(current_container_index_);
            msg::publish(outputs_[0], msg);
            ++current_container_index_;
            return;

        } else {
            current_container_msg_.reset();
        }
    }

    if(provider_) {
        if(provider_->hasNext()) {
            sendToken();
            // we have a message -> return
            return;
        }
    }

    // no message was available
    if(directory_import_) {
        if(provider_) {
            apex_assert(!provider_->hasNext());
            if(isPlaying()) {
                advanceDirectory();
            }
        }

        if(!provider_ || !provider_->hasNext()) {
            createProviderForNextFile();
        }

        if(provider_ && provider_->hasNext()) {
            sendToken();
        }
    }
}

void FileImporter::sendToken()
{
    apex_assert(provider_);
    apex_assert(provider_->hasNext());

    for(std::size_t slot = 0, total = provider_->slotCount(); slot < total; ++slot) {
        INTERLUDE_STREAM("slot " << slot);

        Output* output = outputs_[slot];

        if(msg::isConnected(output)) {
            Message::Ptr msg = provider_->next(slot);

            if(split_container_messages_ && slot == 0) {
                if(GenericVectorMessage::ConstPtr vector = std::dynamic_pointer_cast<GenericVectorMessage>(msg)) {
                    current_container_msg_ = vector;
                    current_container_index_ = 0;
                    process();
                    return;
                }
            }

            if(msg) {
                msg::publish(output, msg);
            }
        }
    }
}

namespace {
bool is_not_digit(char c)
{
    return !std::isdigit(c);
}

bool numeric_string_compare(const std::string& s1, const std::string& s2)
{
    boost::regex re("(.*)_([\\d]+)\\.[^\\.]+");
    boost::smatch result1, result2;

    if (!boost::regex_search(s1, result1, re)) {
        return false;
    }
    if (!boost::regex_search(s2, result2, re)) {
        return false;
    }
    std::string prefix_n1(result1[1].first, result1[1].second);
    std::string prefix_n2(result2[1].first, result2[1].second);

    if(prefix_n1 != prefix_n2) {
        return prefix_n1 < prefix_n2;

    } else {
        std::string suffix_n1(result1[2].first, result1[2].second);
        std::string suffix_n2(result2[2].first, result2[2].second);

        return atoi(suffix_n1.c_str()) < atoi(suffix_n2.c_str());
    }
}
}	

void FileImporter::doImportDir(const QString &dir_string)
{
    dir_files_.clear();

    bool recursive = readParameter<bool>("recursive import");

    //    directory_filter_ = readParameter<std::string>("directory/filter");
    boost::regex filter_regex(directory_filter_);

    std::function<void(const boost::filesystem::path&)> crawl_dir = [&](const boost::filesystem::path& directory) {
        if(boost::filesystem::exists(directory)) {
            boost::filesystem::directory_iterator dir(directory);
            boost::filesystem::directory_iterator end;
            for(; dir != end; ++dir) {
                boost::filesystem::path path = dir->path();

                if(boost::filesystem::is_directory(path)) {
                    if(recursive) {
                        crawl_dir(path);
                    }
                } else {
                    std::string path_string = path.string();
                    if (boost::regex_match(path_string, filter_regex)) {
                        dir_files_.push_back(path_string);
                    }
                }
            }
        }
    };
    boost::filesystem::path directory(dir_string.toStdString());
    try {
        crawl_dir(directory);
    } catch(const std::exception& e) {
        aerr << e.what() << std::endl;
        return;
    }

    bool sort_numerically = readParameter<bool>("directory/sort_numerically");

    if(sort_numerically) {
        std::sort(dir_files_.begin(), dir_files_.end(), numeric_string_compare);
    } else {
        std::sort(dir_files_.begin(), dir_files_.end());
    }

    param::RangeParameter::Ptr current = getParameter<param::RangeParameter>("directory/current");
    current->set(0);
    current->setMax<int>(dir_files_.size());
}

bool FileImporter::createMessageProvider(const QString& file_path)
{
    INTERLUDE("doImport");
    if(file_path.isEmpty()) {
        node_modifier_->setWarning("no file selected");
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
            node_modifier_->setError(std::string("the file ") + file_path.toStdString() + " couldn't be opened");
            return false;
        }
    }

    if(directory_import_) {
        setParameter("directory/current_file", file_path.toStdString());
    }

    file_ = file_path;
    node_modifier_->setNoError();

    try {
        std::string path_str = path.toStdString();
        auto pos = cache_.find(path_str);
        if(pos != cache_.end()) {
            provider_ = pos->second;
            provider_->begin.disconnectAll();
            provider_->slot_count_changed.disconnectAll();
            provider_->no_more_messages.disconnectAll();

        } else {
            INTERLUDE("createMessageProvider");
            provider_ = MessageProviderManager::createMessageProvider(path.toStdString());
            if(cache_enabled_) {
                cache_[path_str] = provider_;
            }
        }

        provider_->slot_count_changed.connect(std::bind(&FileImporter::updateOutputs, this));

        if(!directory_import_) {
            provider_->begin.connect(std::bind(&FileImporter::triggerSignalBegin, this));
            provider_->no_more_messages.connect(std::bind(&FileImporter::triggerSignalEnd, this));
        }

        {
            INTERLUDE("load");
            provider_->load(path.toStdString());
        }

        if(!directory_import_ || readParameter<bool>("directory/show parameters")) {
            setTemporaryParameters(provider_->getParameters(), std::bind(&FileImporter::updateProvider, this));
        }

        provider_->restart();

        return provider_.get();

    } catch(const std::exception& e) {
        node_modifier_->setNoError();
        throw std::runtime_error(std::string("cannot load file ") + file_.toStdString() + ": " + e.what());
    }

    return false;
}

void FileImporter::updateProvider()
{
    if(provider_) {
        provider_->parameterChanged();
    }
}

void FileImporter::triggerSignalBegin()
{
    signalBegin();
}

void FileImporter::triggerSignalEnd()
{
    trigger_signal_end_ = true;
}

void FileImporter::signalBegin()
{
    msg::trigger(begin_);
}

void FileImporter::signalEnd()
{
    trigger_signal_end_ = false;

    if(!end_triggered_) {
        end_triggered_ = true;
        msg::trigger(end_);

        TokenDataConstPtr end;

        if(quit_on_end_) {
            end = connection_types::makeEmpty<EndOfProgramMessage>();
        } else {
            end = connection_types::makeEmpty<EndOfSequenceMessage>();
        }

        for(auto& o : node_handle_->getExternalOutputs()) {
            msg::publish(o.get(), end);
        }
    }
}

void FileImporter::updateOutputs()
{
    std::size_t slot_count = provider_->slotCount();

    std::size_t output_count = outputs_.size();

    if(slot_count > output_count) {
        for(std::size_t i = output_count ; i < slot_count ; ++i) {
            outputs_.push_back(node_modifier_->addOutput<AnyMessage>("unknown"));
        }
    } else {
        bool del = true;
        for(int i = output_count-1 ; i >= (int) slot_count; --i) {
            Output* output = outputs_[i];
            if(msg::isConnected(output)) {
                del = false;
            }

            if(del) {
                node_modifier_->removeOutput(msg::getUUID(output));
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

void FileImporter::requestImport()
{
    import_requested_ = true;
    yield();
}

void FileImporter::import()
{
    directory_import_ = readParameter<bool>("import directory");
    provider_.reset();

    if(directory_import_) {
        removeTemporaryParameters();
        doImportDir(QString::fromStdString(readParameter<std::string>("directory")));
    } else {
        createMessageProvider(QString::fromStdString(readParameter<std::string>("path")));
    }
}

bool FileImporter::isPlaying()
{
    return readParameter<bool>("directory/play") || (play_->isConnected() && playing_);
}

void FileImporter::advanceDirectory()
{
    int current = readParameter<int>("directory/current");
    if(current == last_directory_index_) {
        ++current;

        //DEBUGainfo << "increasing current to " << current << std::endl;
        setParameter("directory/current", current);
    }
}


void FileImporter::createProviderForNextFile()
{
    bool latch = readParameter<bool>("directory/latch");
    bool loop = readParameter<bool>("directory/loop");

    int current = readParameter<int>("directory/current");
    bool index_changed = current != last_directory_index_;

    if(!isPlaying() && !latch && !index_changed) {
        return;
    }
    INTERLUDE("directory");

    int files = dir_files_.size();

    //DEBUGainfo << "current: " << current << std::endl;
    if(current >= files) {
        if(!end_triggered_) {
            signalEnd();
            return;
        }

        if((play_->isConnected() && playing_)) {
            return;
        }

        if(loop) {
            current = 0;
        } else if(latch && files > 0) {
            current = files - 1;
        } else {
            setParameter("directory/play", false);
        }
    }

    if(current < 0) {
        current = 0;
    }

    if(current < files) {
        if(current == 0) {
            signalBegin();
        }

        if(current < files - 1) {
            end_triggered_ = false;
        }

        createMessageProvider(QString::fromStdString(dir_files_.at(current)));
        msg::trigger(new_provider_);

        last_directory_index_ = current;

        //DEBUGainfo << "setting current to " << current << std::endl;
        setParameter("directory/current", current);
        //            int current = readParameter<int>("directory/current");
    }
}
