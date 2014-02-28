#ifndef SEQUENCE_MEAN_H
#define SEQUENCE_MEAN_H

/// COMPONENT
#include <csapex/model/node.h>
#include <opencv2/core/core.hpp>
#include <deque>

namespace csapex {
class SequenceMean : public csapex::Node
{
public:
    SequenceMean();

    virtual void process();
    virtual void setup();

protected:
    ConnectorIn  *input_;
    ConnectorOut *output_;

    int                     type_;
    int                     channels_;
    cv::Size                size_;
    std::deque<cv::Mat>     acc_;

    unsigned int            sequence_size_;

    void update();
    bool check(const cv::Mat &mat);
};
}

#endif // SEQUENCE_MEAN_H
