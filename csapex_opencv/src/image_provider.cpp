/// HEADER
#include <csapex_opencv/image_provider.h>

/// COMPONENT
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

using namespace csapex;

std::map<std::string, ImageProvider::ProviderConstructor> ImageProvider::plugins;

ImageProvider::ImageProvider()
{
    setType(makeEmpty<connection_types::CvMatMessage>());
}

ImageProvider::~ImageProvider()
{
}

void ImageProvider::init()
{
    doInit();
}

connection_types::Message::Ptr ImageProvider::next(std::size_t slot)
{
    cv::Mat mask;

    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::unknown, "unknown", 0));
    next(msg->value, mask);
    msg->setEncoding((msg->value.channels() == 1) ? enc::mono : enc::bgr);

    return msg;
}

std::vector<std::string> ImageProvider::getExtensions() const
{
    return std::vector<std::string>();
}

int ImageProvider::sleepTime()
{
    return 100;
}

GenericStatePtr ImageProvider::getState() const
{
    GenericState::Ptr r(new GenericState(state));
    return r;
}

void ImageProvider::setParameterState(GenericStatePtr memento)
{
    std::shared_ptr<GenericState> m = std::dynamic_pointer_cast<GenericState>(memento);
    if (m) {
        state.setFrom(*m);
    }
}
