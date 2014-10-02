#ifndef JANN_FORMAT_EXPORTER_H
#define JANN_FORMAT_EXPORTER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <boost/thread/mutex.hpp>

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

    void save();
    void clear();

private:
    Input*                                              in_;
    Input*                                              in_vector_;

    boost::mutex                                        m_;
    std::vector<connection_types::FeaturesMessage>      msgs_;

};
}


#endif // JANN_FORMAT_EXPORTER_H
