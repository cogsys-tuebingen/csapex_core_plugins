#ifndef FOREACH_ADAPTER_H
#define FOREACH_ADAPTER_H

/// PROJECT
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include "foreach.h"

namespace csapex {

class ForeachAdapter : public QObject, public NodeAdapter
{
public:
    ForeachAdapter(Foreach *node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

protected:
    Foreach* wrapped_;
};

}

#endif // FOREACH_ADAPTER_H
