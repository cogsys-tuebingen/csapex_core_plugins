/// HEADER
#include "vector_plot_adapter.h"

/// PROJECT
#include <csapex/model/node_facade_local.h>

/// SYSTEM
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <qwt_legend_label.h>

using namespace csapex;

CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(VectorPlotAdapter, csapex::VectorPlot)


VectorPlotAdapter::VectorPlotAdapter(NodeFacadeLocalPtr worker, NodeBox* parent, std::weak_ptr<VectorPlot> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node)
{
    auto n = wrapped_.lock();
    observe(n ->display_request, this, &VectorPlotAdapter::displayRequest);
    observe(n ->update, this, &VectorPlotAdapter::displayRequest);
}

void VectorPlotAdapter::setupUi(QBoxLayout* layout)
{
    auto n = wrapped_.lock();

    plot_widget_ = new QwtPlot;
    plot_widget_->setFixedSize(n->getWidth(), n->getHeight());
    layout->addWidget(plot_widget_);

    DefaultNodeAdapter::setupUi(layout);

    connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

void VectorPlotAdapter::display()
{
    auto n = wrapped_.lock();

    plot_widget_->setFixedSize(n->getWidth(), n->getHeight());

    plot_widget_->detachItems();


    //these getters are blocking. Collect data first then render.
    std::size_t num_curves = n->getVDataCountNumCurves();
    std::size_t num_points = n->getCount();
    n->updateLineColors();
    std::vector<QwtPlotCurve*> curve(num_curves)/*= new QwtPlotCurve[n->getVDataCountNumCurves()];*/;
    std::vector<QColor> colors(num_curves);
    std::vector<QColor> line_colors(num_curves);
    std::vector<const double*> data(num_curves);
    const double* tdata = n->getTData();
    for(std::size_t i = 0; i < num_curves; ++i){
        colors[i] = n->getFillColor();
        line_colors[i] = n->getLineColor(i);
        data[i] = n->getVData(i);
    }

    for(std::size_t i = 0; i < num_curves; ++i){
        curve[i] = new QwtPlotCurve;
        curve[i]->setBaseline(0.0);
        curve[i]->setPen(line_colors[i], n->getLineWidth());
        curve[i]->setStyle(QwtPlotCurve::Lines);

        curve[i]->setBrush(QBrush(colors[i], Qt::SolidPattern));

        curve[i]->setRawSamples(tdata, data[i], num_points);

        curve[i]->attach(plot_widget_);
        plot_widget_->replot();
    }

}


/// MOC
#include "moc_vector_plot_adapter.cpp"

