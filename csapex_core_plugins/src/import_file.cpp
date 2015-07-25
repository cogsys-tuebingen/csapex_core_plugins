/// HEADER
#include "import_file.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/core/settings.h>
#include <csapex/msg/message_factory.h>
#include <csapex/signal/trigger.h>

CSAPEX_REGISTER_CLASS(csapex::ImportFile, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace bfs = boost::filesystem;

ImportFile::ImportFile()
    : prefix_("msg"), do_buffer_(false), next_is_first_(true)
{
}

void ImportFile::setupParameters(Parameterizable& parameters)
{
    addParameter(param::ParameterFactory::declareDirectoryInputPath("path",
                                                                    param::ParameterDescription("Directory to read messages from"),
                                                                    "", ""), std::bind(&ImportFile::setImportPath, this));

    addParameter(param::ParameterFactory::declareText("filename",
                                                      param::ParameterDescription("Base name of the exported messages, suffixed by a counter"),
                                                      "msg"), std::bind(&ImportFile::setImportPrefix, this));
    addParameter(param::ParameterFactory::declareBool("loop",
                                                      param::ParameterDescription("When reaching the end of the directory, do a loop?"),
                                                      true));
    addParameter(param::ParameterFactory::declareBool("buffer",
                                                      param::ParameterDescription("Buffer messages for future rounds."),
                                                      false), [this](param::Parameter* p) {
        do_buffer_ = p->as<bool>();
    });
    param::Parameter::Ptr immediate = param::ParameterFactory::declareBool("immediate", false);
    parameters.addParameter(immediate, std::bind(&ImportFile::changeMode, this));
}


void ImportFile::changeMode()
{
    if(readParameter<bool>("immediate")) {
        modifier_->setTickFrequency(-1.0);
    } else {
        modifier_->setTickFrequency(30.0);
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

    begin_ = node_modifier.addTrigger("begin");
    end_ = node_modifier.addTrigger("end");
}

void ImportFile::process()
{
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
            end_->trigger();
            if(readParameter<bool>("loop")) {
                current_file_ = bfs::directory_iterator(path_);
                next_is_first_ = true;
            }
            //            continue_searching = false;

        } else {
            if(bfs::is_directory(current_file_->status())) {
                // do nothing
            } else {
                bfs::path path = current_file_->path();
                if(path.filename().string().substr(0, prefix_.size()) == prefix_) {
                    if(path.extension() == Settings::message_extension) {
                        ConnectionType::ConstPtr msg;

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
