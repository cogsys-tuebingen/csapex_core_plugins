/// HEADER
#include "roc_curve_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QTableView>
#include <QPainter>
#include <QGridLayout>
#include <QLabel>
#include <QApplication>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(ROCCurveAdapter, csapex::ROCCurve)


ROCCurveAdapter::ROCCurveAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<ROCCurve> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node), plot_widget_(nullptr)
{
    auto n = wrapped_.lock();
    // translate to UI thread via Qt signal
    trackConnection(n ->display_request.connect(std::bind(&ROCCurveAdapter::displayRequest, this)));
}

ROCCurveAdapter::~ROCCurveAdapter()
{
}

void ROCCurveAdapter::setupUi(QBoxLayout* layout)
{
    auto n = wrapped_.lock();

    QGridLayout* grid = new QGridLayout;
    layout->addLayout(grid);

    plot_widget_ = new QwtPlot;
    plot_widget_->setFixedSize(n->readParameter<int>("width"),
                               n->readParameter<int>("height"));

    plot_widget_->setAxisAutoScale(QwtPlot::xBottom, false);
    plot_widget_->setAxisAutoScale(QwtPlot::yLeft, false);
    plot_widget_->setAxisScale(QwtPlot::xBottom, 0.0, 1.0);
    plot_widget_->setAxisScale(QwtPlot::yLeft, 0.0, 1.0);


    roc_curve_ = new QwtPlotCurve("ROC");

    roc_curve_->attach(plot_widget_);


    grid->addWidget(plot_widget_, 1, 1);
    connect(this, SIGNAL(displayRequest()), this, SLOT(display()));

    DefaultNodeAdapter::setupUi(layout);
}

void ROCCurveAdapter::display()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    QVector<double> x;
    QVector<double> y;

    if(node->getType() == ROCCurve::Type::ROC) {
        x.push_back(0);
        y.push_back(0);
        for(const ROCCurve::Entry& entry : node->getEntries()) {
            y.push_back(entry.recall);
            x.push_back(entry.specificity);
        }
        x.push_back(1);
        y.push_back(1);

    } else if(node->getType() == ROCCurve::Type::PR) {
        for(const ROCCurve::Entry& entry : node->getEntries()) {
            y.push_back(entry.recall);
            x.push_back(entry.precision);
        }
    }

    roc_curve_->setSamples(x, y);
    plot_widget_->replot();
}
/// MOC
#include "moc_roc_curve_adapter.cpp"
