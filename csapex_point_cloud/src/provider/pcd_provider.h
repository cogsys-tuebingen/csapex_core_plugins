#ifndef PCD_PROVIDER_H
#define PCD_PROVIDER_H

/// PROJECT
#include <csapex/serialization/serializable.h>
#include <csapex/msg/message_provider.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

/// SYSTEM
#include <functional>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN PCDPointCloudMessageProvider : public MessageProvider
{
public:
    using Ptr = std::shared_ptr<PCDPointCloudMessageProvider>;

protected:
    using ProviderConstructor = std::function<PCDPointCloudMessageProvider*(const std::string&)>;

public:
    PCDPointCloudMessageProvider();
    void load(const std::string &file);

public:
    bool hasNext();

    connection_types::Message::Ptr next(std::size_t slot);

    std::vector<std::string> getExtensions() const;

    GenericStatePtr getState() const;
    void setParameterState(GenericStatePtr memento);

private:
    connection_types::PointCloudMessage::Ptr point_cloud_;
    bool sent_;

    static std::map<std::string, ProviderConstructor> plugins;

};
}

#endif // PCD_PROVIDER_H
