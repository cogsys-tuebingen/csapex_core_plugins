#ifndef VECTORPLOTADAPTER_H
#define VECTORPLOTADAPTER_H

#include "vector_plot.h"

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
class VectorPlotAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT
public:
    VectorPlotAdapter(NodeFacadeImplementationPtr worker, NodeBox* parent, std::weak_ptr<VectorPlot> node);

    void setupUi(QBoxLayout* layout) override;

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    std::weak_ptr<VectorPlot> wrapped_;

    QwtPlot* plot_widget_;
};
}  // namespace csapex

#endif  // VECTORPLOTADAPTER_H
