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

private:
    std::string prefix_;
    std::string path_;
    Output* out_;

    boost::filesystem::directory_iterator current_file_;
};


}

#endif // IMPORT_FILE_H
