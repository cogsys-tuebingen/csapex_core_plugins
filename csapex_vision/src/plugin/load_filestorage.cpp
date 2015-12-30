/// HEADER
#include "load_filestorage.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::LoadFilestorage, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

LoadFilestorage::LoadFilestorage()
{

}

LoadFilestorage::~LoadFilestorage()
{

}

void LoadFilestorage::process()
{
}

void LoadFilestorage::tick()
{
    connection_types::CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::unknown, 0));
    bool resend  = readParameter<bool>("resend");
    bool iterate = readParameter<bool>("iterate");

    if(mats_it_ == mats_.end()) {
        if(resend) {
            mats_it_   = mats_.begin();
            mats_last_ = mats_.begin();
        } else {
            return;
        }
    }

    out->value = mats_it_->second.clone();

    if(iterate) {
        mats_last_ = mats_it_;
        ++mats_it_;
    }
    msg::publish(output_, out);

}

bool LoadFilestorage::canTick()
{
    bool resend  = readParameter<bool>("resend");
    bool iterate = readParameter<bool>("iterate");
    if(resend)
        return true;
    if(iterate && mats_it_ != mats_.end()) {
        return true;
    }

    return mats_it_ != mats_last_;
}

void LoadFilestorage::setup(NodeModifier &node_modifier)
{
    output_ = node_modifier.addOutput<CvMatMessage>("Matrix");
}

void LoadFilestorage::setupParameters(Parameterizable &parameters)
{
    addParameter(
    csapex::param::ParameterFactory::declarePath("file storage",
                                         csapex::param::ParameterDescription("Load cv file storage containing name matrices."),
                                         true,
                                         "",
                                         "*",
                                         true),
    std::bind(&LoadFilestorage::loadFile, this));

    std::vector<std::string> empty;
    addParameter(
    csapex::param::ParameterFactory::declareParameterStringSet("entry",
                                                       csapex::param::ParameterDescription("All contained matrices"),
                                                       empty),
    std::bind(&LoadFilestorage::entryChanged, this));

    addParameter(
    csapex::param::ParameterFactory::declareBool("iterate",
                                          csapex::param::ParameterDescription("Iterate all contained matrices."),
                                          false),
    std::bind(&LoadFilestorage::checkChanged, this));

    addParameter(
    csapex::param::ParameterFactory::declareBool("resend",
                                         csapex::param::ParameterDescription("Resend matrix / all matrices."),
                                         true),
    std::bind(&LoadFilestorage::checkChanged, this));
}

void LoadFilestorage::loadFile()
{
    std::string path = readParameter<std::string>("file storage");
    if(path.empty())
        return;

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "Failed to open '" << path << "'!" << std::endl;
        return;
    }

    std::vector<std::string>       names;
    std::map<std::string, cv::Mat> matrices;
    cv::FileNode root = fs.root();
    for(auto it  = root.begin() ;
             it != root.end() ;
           ++it) {
        std::string name = (*it).name();
        names.push_back(name);
        cv::Mat mat;
        try {
        fs[name] >> mat;
        matrices.insert(std::make_pair(name, mat));
        } catch (const cv::Exception &e) {
            std::cerr << e.what() << std::endl;
            std::cerr << "Can only load named matrices!" << std::endl;
        }
    }

    std::swap(mats_, matrices);
    mats_it_   = mats_.begin();
    mats_last_ = mats_.begin();

    param::SetParameter::Ptr param =
            std::dynamic_pointer_cast<param::SetParameter>(getParameter("entry"));
    param->setSet(names);
    param->set(names.front());
}

void LoadFilestorage::entryChanged()
{
    param::ValueParameter::Ptr iterate =
            std::dynamic_pointer_cast<param::ValueParameter>(getParameter("iterate"));
    iterate->set(false);

    std::string entry = readParameter<std::string>("entry");
    mats_it_   = mats_.find(entry);
    mats_last_ = mats_it_;
}

void LoadFilestorage::checkChanged()
{
    if(mats_it_ == mats_.end()) {
        mats_it_   = mats_.begin();
        mats_last_ = mats_.begin();
    }
}
