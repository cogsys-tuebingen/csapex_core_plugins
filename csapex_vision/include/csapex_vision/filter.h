#ifndef FILTER_H
#define FILTER_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <memory>
#include <opencv2/opencv.hpp>

namespace csapex
{

class Input;
class Output;

class Filter : public Node
{
public:
    typedef std::shared_ptr<Filter> Ptr;

public:
    virtual ~Filter();

    void setup();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask) = 0;

    void process();

protected:
    Filter();
    virtual bool usesMask();

protected:
    Input* input_img_;
    Input* input_mask_;

    Output* output_img_;
    Output* output_mask_;
};

} /// NAMESPACE

#endif // FILTER_H
