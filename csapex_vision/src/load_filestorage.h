#ifndef LOADFILESTORAGE_H
#define LOADFILESTORAGE_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
class LoadFilestorage : public Node
{
public:
    LoadFilestorage();
    ~LoadFilestorage();

    void process() override;
    bool canProcess() const override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Output* output_;
    std::map<std::string, cv::Mat> mats_;
    std::map<std::string, cv::Mat>::iterator mats_it_;
    std::map<std::string, cv::Mat>::iterator mats_last_;

    void loadFile();
    void entryChanged();
    void checkChanged();
};
}  // namespace csapex

#endif  // LOADFILESTORAGE_H
