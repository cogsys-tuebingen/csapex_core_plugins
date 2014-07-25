/// HEADER
#include "import_file.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/core/settings.h>
#include <csapex/model/message_factory.h>

CSAPEX_REGISTER_CLASS(csapex::ImportFile, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace bfs = boost::filesystem;

ImportFile::ImportFile()
    : prefix_("msg")
{
}

void ImportFile::setupParameters()
{
    addParameter(param::ParameterFactory::declareDirectoryInputPath("path",
                                                                    param::ParameterDescription("Directory to read messages from"),
                                                                    "", ""), boost::bind(&ImportFile::setImportPath, this));

    addParameter(param::ParameterFactory::declareText("filename",
                                                      param::ParameterDescription("Base name of the exported messages, suffixed by a counter"),
                                                      "msg"), boost::bind(&ImportFile::setImportPrefix, this));
    addParameter(param::ParameterFactory::declareBool("loop",
                                                      param::ParameterDescription("When reaching the end of the directory, do a loop?"),
                                                      true));
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

void ImportFile::setup()
{
    out_ = modifier_->addOutput<connection_types::AnyMessage>("?");
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
            if(readParameter<bool>("loop")) {
                current_file_ = bfs::directory_iterator(path_);
            }
            continue_searching = false;

        } else {
            if(bfs::is_directory(current_file_->status())) {
                // do nothing
            } else {
                bfs::path path = current_file_->path();
                if(path.filename().string().substr(0, prefix_.size()) == prefix_) {
                    if(path.extension() == Settings::message_extension) {
                        ConnectionType::Ptr msg = MessageFactory::readMessage(path.string());
                        out_->setType(msg->toType());
                        out_->publish(msg);
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
