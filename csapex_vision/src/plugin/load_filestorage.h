#ifndef LOADFILESTORAGE_H
#define LOADFILESTORAGE_H

/// PROJECT
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class LoadFilestorage : public TickableNode
{
public:
    LoadFilestorage();
    ~LoadFilestorage();

    virtual void process() override;
    virtual void tick() override;
    virtual bool canTick() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

private:
    Output *output_;
    std::map<std::string, cv::Mat>           mats_;
    std::map<std::string, cv::Mat>::iterator mats_it_;
    std::map<std::string, cv::Mat>::iterator mats_last_;



    void loadFile();
    void entryChanged();
    void checkChanged();
};
}

#endif // LOADFILESTORAGE_H
