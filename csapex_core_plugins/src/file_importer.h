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
    bool canTick();
    void tick();

    bool doImport(const QString& file_path);
    void doImportDir(const QString& dir);

private:
    void changeDirIndex();

    void changeMode();
    void updateProvider();
    void updateOutputs();

private:
    MessageProvider::Ptr provider_;

    Trigger* begin_;
    Trigger* end_;

    bool directory_import_;
    QString file_;
    std::vector<std::string> dir_files_;

    std::vector<Output*> outputs_;
};

}

#endif // FILE_IMPORTER_H
