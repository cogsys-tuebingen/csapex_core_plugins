/// HEADER
#include "matrix_provider.h"

/// PROJECT
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/assign.hpp>
#include <utility>

using namespace csapex;


CSAPEX_REGISTER_CLASS(csapex::MatrixProvider, csapex::MessageProvider)

MatrixProvider::MatrixProvider()
{
    state.addParameter(
    param::ParameterFactory::declareBool("iterate",
                                          param::ParameterDescription("Iterate all contained matrices."),
                                          false));
    std::vector<std::string> empty;
    state.addParameter(
    param::ParameterFactory::declareParameterStringSet("matrix",
                                                       param::ParameterDescription("All contained matrices"),
                                                       empty));
    state.addParameter(
    param::ParameterFactory::declareBool("resend",
                                         param::ParameterDescription("Resend matrix / all matrices."),
                                         true));
}

MatrixProvider::~MatrixProvider()
{

}
void MatrixProvider::load(const std::string &file)
{
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "Failed to open '" << file << "'!" << std::endl;
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
            std::dynamic_pointer_cast<param::SetParameter>(state.getParameter("matrix"));
    param->setSet(names);
    param->set(names.front());
    last_selection_ = names.front();
    setSlotCount(1);
}

bool MatrixProvider::hasNext()
{
    bool resend  = state.readParameter<bool>("resend");
    bool iterate = state.readParameter<bool>("iterate");
    if(resend)
        return true;
    if(iterate && mats_it_ != mats_.end()) {
        return true;
    }

    return mats_it_ != mats_last_;
}

connection_types::Message::Ptr MatrixProvider::next(std::size_t slot)
{
    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::unknown, 0));
    bool resend  = state.readParameter<bool>("resend");
    bool iterate = state.readParameter<bool>("iterate");
    std::string selection = state.readParameter<std::string>("matrix");

    if(selection != last_selection_) {
        param::ValueParameter::Ptr param_iterate =
                std::dynamic_pointer_cast<param::ValueParameter>(state.getParameter("iterate"));
        param_iterate->set(false);
        iterate    = false;
        mats_it_   = mats_.find(selection);
        mats_last_ = mats_it_;
    }

    if(mats_it_ == mats_.end() && resend) {
        mats_it_   = mats_.begin();
        mats_last_ = mats_.begin();
    } else {
        return msg;
    }

    msg->value = mats_it_->second.clone();

    if(iterate) {
        mats_last_ = mats_it_;
        ++mats_it_;
    }

    return msg;
}

std::string MatrixProvider::getLabel(std::size_t slot) const
{
    if(mats_it_ != mats_.end())
        return mats_it_->first;
    return "";
}

std::vector<std::string> MatrixProvider::getExtensions() const
{
    return boost::assign::list_of (".cv")(".cvgz");
}

Memento::Ptr MatrixProvider::getState() const
{
    GenericState::Ptr r(new GenericState(state));
    return r;
}

void MatrixProvider::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<GenericState> m = std::dynamic_pointer_cast<GenericState> (memento);
    if(m) {
        state.setFrom(*m);
    }
}
