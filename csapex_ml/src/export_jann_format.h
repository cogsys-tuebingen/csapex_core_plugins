#ifndef JANN_FORMAT_EXPORTER_H
#define JANN_FORMAT_EXPORTER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <QMutex>

namespace csapex {
class ExportJANNFormat : public Node
{
public:
    ExportJANNFormat();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();

protected:
    void setExportPath();

    void doExport(const connection_types::FeaturesMessage::Ptr& msg);

private:
    Input*              connector_;

    int                 last_classes_;
    int                 last_id_;
    std::vector<int>    id_label_;

    std::string path_;
    std::string base_;
    QMutex      mutex_;

    int         suffix_;
};
}


#endif // JANN_FORMAT_EXPORTER_H
