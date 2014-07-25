#ifndef FILTER_H
#define FILTER_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

namespace csapex
{

class ConnectorIn;
class ConnectorOut;

class Filter : public Node
{
public:
    typedef boost::shared_ptr<Filter> Ptr;

public:
    virtual ~Filter();

    void setup();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask) = 0;

    void process();

protected:
    Filter(const UUID &uuid = UUID::NONE);
    virtual bool usesMask();

protected:
    ConnectorIn* input_img_;
    ConnectorIn* input_mask_;

    ConnectorOut* output_img_;
    ConnectorOut* output_mask_;
};

} /// NAMESPACE

#endif // FILTER_H
