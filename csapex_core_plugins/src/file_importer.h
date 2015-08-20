#ifndef FILE_IMPORTER_H
#define FILE_IMPORTER_H

/// PROJECT
#include <csapex/msg/message_provider.h>
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <QString>

namespace csapex
{

class Output;

class FileImporter : public TickableNode
{
public:
    FileImporter();
    ~FileImporter();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;

    void import();

    virtual void process() override;
    virtual bool canTick() override;
    virtual void tick() override;

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
    int last_directory_index_;
    QString file_;
    std::vector<std::string> dir_files_;

    std::vector<Output*> outputs_;
};

}

#endif // FILE_IMPORTER_H
