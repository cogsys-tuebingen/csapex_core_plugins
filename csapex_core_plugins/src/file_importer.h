#ifndef FILE_IMPORTER_H
#define FILE_IMPORTER_H

/// PROJECT
#include <csapex/msg/message_provider.h>
#include <csapex/model/tickable_node.h>
#include <csapex/signal/signal_fwd.h>

/// SYSTEM
#include <QString>

namespace csapex
{

class FileImporter : public TickableNode
{
public:
    FileImporter();
    ~FileImporter();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;

    void import();

    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters) override;
    virtual bool canTick() override;
    virtual bool tick(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters) override;

    bool createMessageProvider(const QString& file_path);
    void doImportDir(const QString& dir);

private:
    void changeDirIndex();

    void changeMode();
    void updateProvider();
    void updateOutputs();

    void signalEnd();

private:
    MessageProvider::Ptr provider_;

    Slot* play_;
    bool playing_;
    bool abort_;
    bool end_triggered_;

    Trigger* begin_;
    Trigger* end_;

    bool directory_import_;
    int last_directory_index_;
    QString file_;
    std::vector<std::string> dir_files_;

    std::vector<Output*> outputs_;

    bool cache_enabled_;
    std::map<std::string, MessageProvider::Ptr> cache_;
};

}

#endif // FILE_IMPORTER_H
