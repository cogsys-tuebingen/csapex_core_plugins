/// HEADER
#include "import_file.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/core/settings.h>
#include <csapex/factory/message_factory.h>
#include <csapex/signal/event.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImportFile, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace bfs = boost::filesystem;

ImportFile::ImportFile()
    : prefix_("msg"), do_buffer_(false), at_end_(false), next_is_first_(true)
{
}

void ImportFile::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declareDirectoryInputPath("path",
                                                                    csapex::param::ParameterDescription("Directory to read messages from"),
                                                                    "", ""), std::bind(&ImportFile::setImportPath, this));

    addParameter(csapex::param::ParameterFactory::declareText("filename",
                                                      csapex::param::ParameterDescription("Base name of the exported messages, suffixed by a counter"),
                                                      "msg"), std::bind(&ImportFile::setImportPrefix, this));
    addParameter(csapex::param::ParameterFactory::declareBool("loop",
                                                      csapex::param::ParameterDescription("When reaching the end of the directory, do a loop?"),
                                                      true));
    addParameter(csapex::param::ParameterFactory::declareBool("buffer",
                                                      csapex::param::ParameterDescription("Buffer messages for future rounds."),
                                                      false), [this](csapex::param::Parameter* p) {
        do_buffer_ = p->as<bool>();
    });
    csapex::param::Parameter::Ptr immediate = csapex::param::ParameterFactory::declareBool("immediate", false);
    parameters.addParameter(immediate, std::bind(&ImportFile::changeMode, this));

    addParameter(csapex::param::ParameterFactory::declareTrigger("restart"), [this](csapex::param::Parameter*) {
        restart();
    });
}


void ImportFile::changeMode()
{
    if(readParameter<bool>("immediate")) {
        setTickFrequency(-1.0);
    } else {
        setTickFrequency(30.0);
    }
}

void ImportFile::setImportPrefix()
{
    prefix_ = readParameter<std::string>("filename");
}

void ImportFile::setImportPath()
{
    std::string path = readParameter<std::string>("path");

    if(path.empty() || path_ == path) {
        return;
    }

    path_ = path;

    if(!bfs::exists(path_)) {
        path_ = "";
        throw std::runtime_error("path '" + path_ + "' doesn't exist");
    }

    current_file_ = bfs::directory_iterator(path_);
}

void ImportFile::setup(NodeModifier& node_modifier)
{
    out_ = node_modifier.addOutput<connection_types::AnyMessage>("?");

    begin_ = node_modifier.addEvent("begin");
    end_ = node_modifier.addEvent("end");
}

void ImportFile::process()
{
}

void ImportFile::restart()
{
    current_file_ = bfs::directory_iterator(path_);
    next_is_first_ = true;
    at_end_ = false;
}

void ImportFile::tick()
{
    if(path_.empty()) {
        return;
    }

    bfs::directory_iterator end;

    bool continue_searching = true;
    while(continue_searching) {
        if(current_file_ == end) {
            if(!at_end_) {
                at_end_ = true;
                end_->trigger();

            } else {
                continue_searching = false;
            }

        } else if(at_end_){
            if(readParameter<bool>("loop")) {
                restart();

            } else {
                continue_searching = false;
            }

        } else {
            at_end_ = false;
            if(bfs::is_directory(current_file_->status())) {
                // do nothing
            } else {
                bfs::path path = current_file_->path();
                if(path.filename().string().substr(0, prefix_.size()) == prefix_) {
                    if(path.extension() == Settings::message_extension) {
                        Token::ConstPtr msg;

                        bool buffered = false;
                        if(do_buffer_) {
                            auto pos = buffer_.find(path.string());
                            buffered = pos != buffer_.end();
                            if(buffered) {
                                msg = pos->second;
                            }
                        }

                        if(!msg) {
                            msg = MessageFactory::readMessage(path.string());
                        }

                        if(do_buffer_ && !buffered) {
                            buffer_[path.string()] = msg;
                        }

                        if(next_is_first_) {
                            next_is_first_ = false;
                            begin_->trigger();
                        }


                        msg::publish(out_, msg);
                        continue_searching = false;
                    } else {
                        aerr << "cannot handle type of file " << path << std::endl;
                    }
                }
            }
            ++current_file_;
        }
    }
}
