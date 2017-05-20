/// HEADER
#include "scatter_plot_adapter.h"

/// SYSTEM
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <qwt_legend_label.h>

using namespace csapex;

CSAPEX_REGISTER_LEGACY_NODE_ADAPTER(ScatterPlotAdapter, csapex::ScatterPlot)


ScatterPlotAdapter::ScatterPlotAdapter(NodeFacadeWeakPtr worker, NodeBox* parent, std::weak_ptr<ScatterPlot> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node)
{
    auto n = wrapped_.lock();
    trackConnection(n ->display_request.connect(std::bind(&ScatterPlotAdapter::displayRequest, this)));
    trackConnection(n ->update.connect(std::bind(&ScatterPlotAdapter::displayRequest, this)));
}

void ScatterPlotAdapter::setupUi(QBoxLayout* layout)
{
    auto n = wrapped_.lock();

    plot_widget_ = new QwtPlot;
    plot_widget_->setFixedSize(n->getWidth(), n->getHeight());
    layout->addWidget(plot_widget_);

    DefaultNodeAdapter::setupUi(layout);

    connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

void ScatterPlotAdapter::display()
{
    auto n = wrapped_.lock();

    plot_widget_->setFixedSize(n->getWidth(), n->getHeight());

    plot_widget_->detachItems();

    QwtPlotCurve* curve = new QwtPlotCurve;
    curve->setBaseline(0.0);
    curve->setStyle(QwtPlotCurve::Dots);
    curve->setPen(n->getFillColor(), n->readParameter<double>("point_size"));

    curve->setRawSamples(n->getXData(), n->getYData(), n->getCount());

    curve->scaleRect(n->getXMap(), n->getYMap());

    curve->attach(plot_widget_);

    plot_widget_->replot();
}


/// MOC
#include "moc_scatter_plot_adapter.cpp"

