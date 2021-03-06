#ifndef CAMERA_H_
#define CAMERA_H_

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

/// HEADER
#include <csapex/msg/io.h>

namespace csapex
{
class Camera : public Node
{
public:
    Camera();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

    bool canProcess() const override;

protected:
    void update();

private:
    Output* output_;
    cv::VideoCapture cap_;

    int current_dev_;
    int w_;
    int h_;
};

}  // namespace csapex

#endif  // CAMERA_H_
