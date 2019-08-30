/// HEADER
#include "time_plot_adapter.h"

/// PROJECT
#include <csapex/model/node_facade_impl.h>

/// SYSTEM
#include <libqwt/qwt_legend.h>
#include <libqwt/qwt_legend_label.h>
#include <libqwt/qwt_plot.h>
#include <libqwt/qwt_plot_curve.h>

using namespace csapex;

CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(TimePlotAdapter, csapex::TimePlot)

TimePlotAdapter::TimePlotAdapter(NodeFacadeImplementationPtr node_facade, NodeBox* parent, std::weak_ptr<TimePlot> node) : DefaultNodeAdapter(node_facade, parent), wrapped_(node)
{
    auto n = wrapped_.lock();
    observe(n->display_request, this, &TimePlotAdapter::displayRequest);
    observe(n->update, this, &TimePlotAdapter::displayRequest);
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

    plot_widget_->detachItems();

    std::size_t n_curves = n->getVDataCountNumCurves();
    std::vector<QwtPlotCurve*> curve;
    curve.resize(n_curves);
    for (std::size_t i = 0; i < n_curves; ++i) {
        curve[i] = new QwtPlotCurve;
        curve[i]->setBaseline(0.0);
        curve[i]->setPen(n->getLineColor(i), n->getLineWidth());
        curve[i]->setStyle(QwtPlotCurve::Lines);

        curve[i]->setBrush(QBrush(n->getFillColor(), Qt::SolidPattern));

        curve[i]->setRawSamples(n->getTData(), n->getVData(i), n->getCount());

        curve[i]->attach(plot_widget_);
        plot_widget_->replot();
    }
}

/// MOC
#include "moc_time_plot_adapter.cpp"
