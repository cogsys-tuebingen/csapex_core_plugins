#ifndef POINT_COUNT_ADAPTER_H
#define POINT_COUNT_ADAPTER_H

/// PROJECT
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include "point_count.h"

/// SYSTEM
#include <QLCDNumber>

namespace csapex {

class PointCountAdapter : public QObject, public NodeAdapter
{
public:
    PointCountAdapter(PointCount *node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

    void display(int img);

protected:
    PointCount* wrapped_;

private:
    QLCDNumber* number_;
};

}

#endif // POINT_COUNT_ADAPTER_H
