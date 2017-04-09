#ifndef ROC_CURVE_ADAPTER_H
#define ROC_CURVE_ADAPTER_H


/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "roc_curve.h"

/// SYSTEM
#include <QWidget>
#include <qwt_plot_curve.h>
#include <qwt_plot_scaleitem.h>
#include <qwt_scale_map.h>

namespace csapex {


class ROCCurveAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    ROCCurveAdapter(NodeFacadeWeakPtr worker, NodeBox* parent, std::weak_ptr<ROCCurve> node);
    ~ROCCurveAdapter();

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    std::weak_ptr<ROCCurve> wrapped_;

    QwtPlot* plot_widget_ ;
    QwtPlotCurve *roc_curve_;
};

}
#endif // ROC_CURVE_ADAPTER_H
