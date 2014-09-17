#ifndef FILE_IMPORTER_H
#define FILE_IMPORTER_H

/// PROJECT
#include <csapex/msg/message_provider.h>
#include <csapex/model/node.h>

namespace csapex
{

class Output;

class FileImporter : public Node
{
public:
    FileImporter();
    ~FileImporter();

    void setupParameters();
    void setup();

    void import();

    void process();

    void tick();

    bool doImport(const QString& path);

private:
    void changeMode();
    void updateProvider();

private:
    MessageProvider::Ptr provider_;

    QString file_;
    Output* output_;
};

}

#endif // FILE_IMPORTER_H
