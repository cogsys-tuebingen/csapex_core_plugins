#ifndef EXPORT_FILE_H_
#define EXPORT_FILE_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <QMutex>

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

private:
    Input* connector_;

    std::string path_;
    std::string base_;
    QMutex mutex_;

    int suffix_;
};

}

#endif // EXPORT_FILE_H_
