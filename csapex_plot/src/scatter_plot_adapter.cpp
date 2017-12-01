/// HEADER
#include "scatter_plot_adapter.h"

/// PROJECT
#include <csapex/model/node_facade_impl.h>

/// SYSTEM
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <qwt_legend_label.h>

using namespace csapex;

CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(ScatterPlotAdapter, csapex::ScatterPlot)


ScatterPlotAdapter::ScatterPlotAdapter(NodeFacadeImplementationPtr worker, NodeBox* parent, std::weak_ptr<ScatterPlot> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node)
{
    auto n = wrapped_.lock();
    observe(n ->display_request, this, &ScatterPlotAdapter::displayRequest);
    observe(n ->update, this, &ScatterPlotAdapter::displayRequest);
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

    std::size_t n_plots = n->getNumberOfPlots();
    n->updateLineColors();
    std::vector<QwtPlotCurve*> curve(n_plots);
    for(std::size_t i = 0; i < n_plots; ++ i){
        curve[i] = new QwtPlotCurve;
    }

    curve[0]->setBaseline(0.0);
    curve[0]->setStyle(QwtPlotCurve::Dots);
    curve[0]->setPen(n->getLineColor(0), n->readParameter<double>("point_size"));

    curve[0]->setRawSamples(n->getXData(), n->getYData(), n->getCount());

    curve[0]->scaleRect(n->getXMap(), n->getYMap());

    curve[0]->attach(plot_widget_);

    plot_widget_->replot();

    for(std::size_t i = 1; i < n_plots; ++i){
        curve[i]->setBaseline(0.0);
        curve[i]->setStyle(QwtPlotCurve::Dots);
        curve[i]->setPen(n->getLineColor(i), n->readParameter<double>("point_size"));

        curve[i]->setRawSamples(n->getXData(), n->getVarData(i-1), n->getCount());

        curve[i]->scaleRect(n->getXMap(), n->getYMap());

        curve[i]->attach(plot_widget_);
        plot_widget_->replot();
    }

}


/// MOC
#include "moc_scatter_plot_adapter.cpp"

