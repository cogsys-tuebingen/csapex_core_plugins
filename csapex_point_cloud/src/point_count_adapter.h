#ifndef POINT_COUNT_ADAPTER_H
#define POINT_COUNT_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "point_count.h"

/// SYSTEM
#include <QLCDNumber>

namespace csapex {

class PointCountAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    PointCountAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<PointCount> node);

    virtual void setupUi(QBoxLayout* layout);

    void display(int img);

Q_SIGNALS:
    void displayRequest(int no);

protected:
    std::weak_ptr<PointCount> wrapped_;

private:
    QLCDNumber* number_;
};

}

#endif // POINT_COUNT_ADAPTER_H
