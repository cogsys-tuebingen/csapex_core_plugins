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

    virtual QIcon getIcon() const;

protected:
    void setImportPath();

private:
    std::string path_;
    ConnectorOut* out_;

    boost::filesystem::directory_iterator current_file_;
};


}

#endif // IMPORT_FILE_H
