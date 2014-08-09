#ifndef IMAGE_PROVIDER_H
#define IMAGE_PROVIDER_H

/// PROJECT
#include <csapex/model/memento.h>
#include <csapex/msg/message_provider.h>

/// SYSTEM
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <map>
#include <opencv2/opencv.hpp>
#include <QBoxLayout>
#include <QFrame>
#include <string>
#include <vector>


namespace csapex
{

class ImageProvider : public MessageProvider
{
public:
    typedef boost::shared_ptr<ImageProvider> Ptr;

protected:
    typedef boost::function<ImageProvider*(const std::string&)> ProviderConstructor;

public:
    ImageProvider();
    virtual ~ImageProvider();

public:
    virtual connection_types::Message::Ptr next();

    std::vector<std::string> getExtensions() const;

public:
    void init();
    virtual void doInit() {}
    virtual bool hasNext() = 0;
    virtual void next(cv::Mat&, cv::Mat&) = 0;
    virtual int sleepTime();

    Memento::Ptr getState() const;
    void setParameterState(Memento::Ptr memento);

private:
    static std::map<std::string, ProviderConstructor> plugins;

};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_H
