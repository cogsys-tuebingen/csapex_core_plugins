#ifndef SCATTER_PLOT_ADAPTER_H
#define SCATTER_PLOT_ADAPTER_H

/// COMPONENT
#include "scatter_plot.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>
#include <csapex/view/node/default_node_adapter.h>

/// SYSTEM
#include <QBoxLayout>

namespace csapex {

class ScatterPlotAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    ScatterPlotAdapter(NodeFacadeLocalPtr worker, NodeBox* parent, std::weak_ptr<ScatterPlot> node);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    std::weak_ptr<ScatterPlot> wrapped_;

    QwtPlot* plot_widget_ ;
};

}

#endif // SCATTER_PLOT_ADAPTER_H
