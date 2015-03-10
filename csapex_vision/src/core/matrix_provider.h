#ifndef MATRIXPROVIDER_H
#define MATRIXPROVIDER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>

/// PROJECT
#include <utils_param/set_parameter.h>

/// SYSTEM
#include <opencv2/core/core.hpp>

namespace csapex {
class MatrixProvider : public MessageProvider
{
public:
    MatrixProvider();
    ~MatrixProvider();

    void load(const std::string& file);

public:
    virtual bool hasNext();
    virtual connection_types::Message::Ptr next(std::size_t slot);
    virtual std::string getLabel(std::size_t slot) const;

    virtual std::vector<std::string> getExtensions() const;

    virtual Memento::Ptr getState() const;
    virtual void setParameterState(Memento::Ptr memento);

private:
    std::string                              file_;
    std::map<std::string, cv::Mat>           mats_;
    std::map<std::string, cv::Mat>::iterator mats_it_;
    std::map<std::string, cv::Mat>::iterator mats_last_;
    std::string                              last_selection_;

    void itemSelected();
};
}

#endif // MATRIXPROVIDER_H
