#ifndef FILTER_STATIC_MASK_ADAPTER_H
#define FILTER_STATIC_MASK_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "filter_static_mask.h"

/// SYSTEM
#include <QGraphicsView>

namespace csapex {

class FilterStaticMaskAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    FilterStaticMaskAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<FilterStaticMask> node);
    ~FilterStaticMaskAdapter();

Q_SIGNALS:
    void displayRequest();
    void inputRequest(cv::Mat);

public Q_SLOTS:
    void display();
    void setMask(cv::Mat);

private:
    std::weak_ptr<FilterStaticMask> wrapped_;
};

}


#endif // FILTER_STATIC_MASK_ADAPTER_H
