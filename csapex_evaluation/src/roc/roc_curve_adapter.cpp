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

    const std::map<double, ROCCurve::Entry>& entries = node->getEntries();

    QVector<double> fpr;
    QVector<double> tpr;

    for(const auto& pair : entries) {
        const ROCCurve::Entry& entry = pair.second;
        tpr.push_back(entry.tpr);
        fpr.push_back(entry.fpr);
    }

    roc_curve_->setSamples(fpr, tpr);
    plot_widget_->replot();
}
/// MOC
#include "moc_roc_curve_adapter.cpp"
