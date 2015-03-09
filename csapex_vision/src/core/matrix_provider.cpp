/// HEADER
#include "matrix_provider.h"

/// PROJECT
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <boost/assign.hpp>
#include <utility>

using namespace csapex;

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
        fs[name] >> mat;
        matrices.insert(std::make_pair(name, mat));
    }

    std::swap(mats_, matrices);
    mats_it_ = mats_.begin();
    mats_last_ = mats_.begin();
    auto param = state.getParameter("matrix");
    param->set(names);
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

    next(msg->value, mask);

    msg->setEncoding((msg->value.channels() == 1) ? enc::mono : enc::bgr);

    return msg;
}

std::string MatrixProvider::getLabel(std::size_t slot) const
{

}

std::vector<std::string> MatrixProvider::getExtensions() const
{
    const static std::vector<std::string> extensions =
            boost::assign::list_of
            (".cv")
            (".cv.tar.gz");
    return extensions;
}

Memento::Ptr MatrixProvider::getState() const
{

}

void MatrixProvider::setParameterState(Memento::Ptr memento)
{

}

void MatrixProvider::parameterChanged()
{

}
