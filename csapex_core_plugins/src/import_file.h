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

    void setupParameters();
    void setup();
    void process();
    void tick();

protected:
    void setImportPath();
    void setImportPrefix();

private:
    std::string prefix_;
    std::string path_;
    Output* out_;

    boost::filesystem::directory_iterator current_file_;
};


}

#endif // IMPORT_FILE_H
