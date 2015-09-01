#ifndef JANN_FORMAT_EXPORTER_H
#define JANN_FORMAT_EXPORTER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <mutex>

namespace csapex {
class ArffFormatExport : public Node
{
public:
    ArffFormatExport();

    virtual void setup(NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

protected:
    void setExportPath();

    void save();
    void clear();

private:
    Input*                                          in_;
    Input*                                          in_vector_;

    std::mutex                                      m_;
    std::set<int>                                   msgs_classes_;
    std::vector<connection_types::FeaturesMessage>  msgs_;

};
}


#endif // JANN_FORMAT_EXPORTER_H
