#ifndef EXPORT_FILE_H_
#define EXPORT_FILE_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>
#include <csapex_core_plugins/vector_message.h>

namespace csapex {

class ExportFile : public Node
{
public:
    ExportFile();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();

protected:
    void setExportPath();

    void exportVector(const connection_types::VectorMessage::ConstPtr& vector);
    void exportSingle(const ConnectionType::ConstPtr& msg);

private:
    Input* connector_;

    std::string path_;
    std::string base_;

    int suffix_;
};

}

#endif // EXPORT_FILE_H_
