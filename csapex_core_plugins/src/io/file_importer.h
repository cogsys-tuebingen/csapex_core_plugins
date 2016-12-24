#ifndef FILE_IMPORTER_H
#define FILE_IMPORTER_H

/// PROJECT
#include <csapex/msg/message_provider.h>
#include <csapex/model/tickable_node.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <QString>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN FileImporter : public Node
{
public:
    FileImporter();
    ~FileImporter();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;

    void requestImport();
    void import();

    virtual void process() override;
    virtual bool canProcess() const override;

    bool createMessageProvider(const QString& file_path);
    void doImportDir(const QString& dir);


private:
    void changeDirIndex();

    void changeMode();
    void updateProvider();
    void updateOutputs();

    void triggerSignalBegin();
    void triggerSignalEnd();

    void signalBegin();
    void signalEnd();

    bool isPlaying();

    void sendToken();

    void advanceDirectory();
    void createProviderForNextFile();

private:
    MessageProvider::Ptr provider_;

    Slot* play_;
    bool playing_;
    bool abort_;
    bool end_triggered_;
    bool quit_on_end_;

    bool trigger_signal_end_;

    bool import_requested_;

    Event* begin_;
    Event* end_;
    Event* new_provider_;

    bool directory_import_;
    std::string directory_filter_;
    int last_directory_index_;
    QString file_;
    std::vector<std::string> dir_files_;

    std::vector<Output*> outputs_;

    bool cache_enabled_;
    std::map<std::string, MessageProvider::Ptr> cache_;

    bool split_container_messages_;
    std::size_t current_container_index_;
    connection_types::GenericVectorMessage::ConstPtr current_container_msg_;
};

}

#endif // FILE_IMPORTER_H
