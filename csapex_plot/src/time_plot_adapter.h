#ifndef TIME_PLOT_ADAPTER_H
#define TIME_PLOT_ADAPTER_H

/// COMPONENT
#include "time_plot.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>
#include <csapex/view/node/default_node_adapter.h>

/// SYSTEM
#include <QBoxLayout>

namespace csapex {

class TimePlotAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    TimePlotAdapter(NodeWorkerWeakPtr worker, std::weak_ptr<TimePlot> node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    std::weak_ptr<TimePlot> wrapped_;

    QwtPlot* plot_widget_ ;
};

}

#endif // TIME_PLOT_ADAPTER_H
