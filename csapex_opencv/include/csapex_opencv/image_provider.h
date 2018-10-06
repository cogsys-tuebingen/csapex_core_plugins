#ifndef IMAGE_PROVIDER_H
#define IMAGE_PROVIDER_H

/// PROJECT
#include <csapex/msg/message_provider.h>
#include <csapex/serialization/serializable.h>

/// SYSTEM
#include <QBoxLayout>
#include <QFrame>
#include <boost/algorithm/string.hpp>
#include <boost/signals2.hpp>
#include <functional>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN ImageProvider : public MessageProvider
{
public:
    typedef std::shared_ptr<ImageProvider> Ptr;

protected:
    typedef std::function<ImageProvider*(const std::string&)> ProviderConstructor;

public:
    ImageProvider();
    virtual ~ImageProvider();

public:
    virtual connection_types::Message::Ptr next(std::size_t slot);

    std::vector<std::string> getExtensions() const;

public:
    void init();
    virtual void doInit()
    {
    }
    virtual void next(cv::Mat&, cv::Mat&) = 0;
    virtual int sleepTime();

    GenericStatePtr getState() const;
    void setParameterState(GenericStatePtr memento);

private:
    static std::map<std::string, ProviderConstructor> plugins;
};

}  // namespace csapex

#endif  // IMAGE_PROVIDER_H
