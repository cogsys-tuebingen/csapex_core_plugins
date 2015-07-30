#ifndef IMPORT_FILE_H
#define IMPORT_FILE_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace csapex {


class ImportFile : public csapex::Node
{
public:
    ImportFile();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;
    void tick();

protected:
    void setImportPath();
    void setImportPrefix();
    void changeMode();
    void restart();

private:
    Output* out_;

    Trigger* begin_;
    Trigger* end_;

    std::string prefix_;
    std::string path_;

    boost::filesystem::directory_iterator current_file_;

    bool do_buffer_;
    std::map<std::string, ConnectionTypeConstPtr> buffer_;

    bool at_end_;
    bool next_is_first_;
};


}

#endif // IMPORT_FILE_H
