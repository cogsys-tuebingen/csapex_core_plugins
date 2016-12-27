#ifndef ARFFFILEIMPORTER_H
#define ARFFFILEIMPORTER_H

/// PROJECT
#include <csapex/model/memento.h>
#include <csapex/msg/message_provider.h>

/// SYSTEM
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <cslibs_arff/arff_data.h>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN ARFFFeatureMessageProvider : public MessageProvider
{
public:
    typedef std::shared_ptr<ARFFFeatureMessageProvider> Ptr;

protected:
    typedef std::function<ARFFFeatureMessageProvider*(const std::string&)> ProviderConstructor;

public:
    ARFFFeatureMessageProvider();
    void load(const std::string& arff_file);

public:
    bool hasNext();

    connection_types::Message::Ptr next(std::size_t slot);

    void next(std::vector<float> &value,
              int &classification);

    std::vector<std::string> getExtensions() const;

    Memento::Ptr getState() const;
    void setParameterState(Memento::Ptr memento);

private:
    cslibs_arff::ArffData::Ptr arff_;

    std::vector<float> last_data_;
    int                last_classification_;

    int                features_messages_;
    int                next_feature_message_;

    static std::map<std::string, ProviderConstructor> plugins;

    void setPlaying(bool playing);
};
}

#endif // ARFFFILEIMPORTER_H
