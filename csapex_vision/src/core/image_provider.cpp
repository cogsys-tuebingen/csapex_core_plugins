/// HEADER
#include <csapex_vision/image_provider.h>

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

using namespace csapex;

std::map<std::string, ImageProvider::ProviderConstructor> ImageProvider::plugins;

ImageProvider::ImageProvider()
{
    setType(connection_types::makeEmpty<connection_types::CvMatMessage>());

    state.addParameter(param::ParameterFactory::declareBool("playback/resend", true));
}

ImageProvider::~ImageProvider()
{;
}


void ImageProvider::init()
{
    doInit();
}

connection_types::Message::Ptr ImageProvider::next()
{
    cv::Mat mask;

    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::unknown));

    next(msg->value, mask);

    msg->setEncoding((msg->value.channels() == 1) ? enc::mono : enc::bgr);

    return msg;
}

std::vector<std::string> ImageProvider::getExtensions() const
{
    return std::vector<std::string> ();
}

int ImageProvider::sleepTime()
{
    return 100;
}


Memento::Ptr ImageProvider::getState() const
{
    GenericState::Ptr r(new GenericState(state));
    return r;
}

void ImageProvider::setParameterState(Memento::Ptr memento)
{
    boost::shared_ptr<GenericState> m = boost::dynamic_pointer_cast<GenericState> (memento);
    if(m) {
        state.setFrom(*m);
    }
}
