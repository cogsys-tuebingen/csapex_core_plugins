#ifndef TIME_PLOT_ADAPTER_H
#define TIME_PLOT_ADAPTER_H

/// COMPONENT
#include "time_plot.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QBoxLayout>

namespace csapex
{
class TimePlotAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    TimePlotAdapter(NodeFacadeImplementationPtr node_facade, NodeBox* parent, std::weak_ptr<TimePlot> node);

    void setupUi(QBoxLayout* layout) override;

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    std::weak_ptr<TimePlot> wrapped_;

    QwtPlot* plot_widget_;
};

}  // namespace csapex

#endif  // TIME_PLOT_ADAPTER_H
