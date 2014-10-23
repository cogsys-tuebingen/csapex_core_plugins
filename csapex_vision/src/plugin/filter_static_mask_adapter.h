#ifndef FILTER_STATIC_MASK_ADAPTER_H
#define FILTER_STATIC_MASK_ADAPTER_H

/// PROJECT
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include "filter_static_mask.h"

/// SYSTEM
#include <QGraphicsView>

namespace csapex {

class FilterStaticMaskAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    FilterStaticMaskAdapter(NodeWorker *worker, FilterStaticMask *node, WidgetController *widget_ctrl);
    ~FilterStaticMaskAdapter();

Q_SIGNALS:
    void displayRequest();
    void inputRequest(cv::Mat);

public Q_SLOTS:
    void display();
    void setMask(cv::Mat);

private:
    FilterStaticMask* wrapped_;
};

}


#endif // FILTER_STATIC_MASK_ADAPTER_H
