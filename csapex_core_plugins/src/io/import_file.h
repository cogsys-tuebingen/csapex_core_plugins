#ifndef IMPORT_FILE_H
#define IMPORT_FILE_H

/// PROJECT
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace csapex {


class ImportFile : public csapex::TickableNode
{
public:
    ImportFile();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;
    virtual void tick() override;

protected:
    void setImportPath();
    void setImportPrefix();
    void changeMode();
    void restart();

private:
    Output* out_;

    Event* begin_;
    Event* end_;

    std::string prefix_;
    std::string path_;

    boost::filesystem::directory_iterator current_file_;

    bool do_buffer_;
    std::map<std::string, TokenDataConstPtr> buffer_;

    bool at_end_;
    bool next_is_first_;
};


}

#endif // IMPORT_FILE_H
