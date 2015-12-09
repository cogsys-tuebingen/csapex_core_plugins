/// HEADER
#include "time_plot_adapter.h"

/// SYSTEM
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(TimePlotAdapter, csapex::TimePlot)


TimePlotAdapter::TimePlotAdapter(NodeHandleWeakPtr worker, std::weak_ptr<TimePlot> node, WidgetController *widget_ctrl)
    : DefaultNodeAdapter(worker, widget_ctrl), wrapped_(node)
{
    auto n = wrapped_.lock();
    trackConnection(n ->display_request.connect(std::bind(&TimePlotAdapter::displayRequest, this)));
    trackConnection(n ->update.connect(std::bind(&TimePlotAdapter::displayRequest, this)));
}

void TimePlotAdapter::setupUi(QBoxLayout* layout)
{
    auto n = wrapped_.lock();

    plot_widget_ = new QwtPlot;
    plot_widget_->setFixedSize(n->getWidth(), n->getHeight());
    layout->addWidget(plot_widget_);

    DefaultNodeAdapter::setupUi(layout);

    connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

void TimePlotAdapter::display()
{
    auto n = wrapped_.lock();

    plot_widget_->setFixedSize(n->getWidth(), n->getHeight());

    QwtPlotCurve* curve = new QwtPlotCurve;
    curve->setBaseline(0.0);

    curve->setPen(n->getLineColor(), n->getLineWidth());
    curve->setStyle(QwtPlotCurve::Lines);

    curve->setBrush(QBrush(n->getFillColor(), Qt::SolidPattern));

    curve->setRawSamples(n->getTData(), n->getVData(), n->getCount());

    plot_widget_->detachItems();
    curve->attach(plot_widget_);

    plot_widget_->replot();
}


/// MOC
#include "moc_time_plot_adapter.cpp"

