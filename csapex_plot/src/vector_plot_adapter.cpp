/// HEADER
#include "vector_plot_adapter.h"

/// SYSTEM
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt/qwt_legend.h>
#include <qwt_legend_label.h>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(VectorPlotAdapter, csapex::VectorPlot)


VectorPlotAdapter::VectorPlotAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<VectorPlot> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node)
{
    auto n = wrapped_.lock();
    trackConnection(n ->display_request.connect(std::bind(&VectorPlotAdapter::displayRequest, this)));
    trackConnection(n ->update.connect(std::bind(&VectorPlotAdapter::displayRequest, this)));
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


    QwtPlotCurve* curve[n->getVDataCountNumCurves()] /*= new QwtPlotCurve[n->getVDataCountNumCurves()];*/;
    for(std::size_t i = 0; i < n->getVDataCountNumCurves(); ++i){
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
#include "moc_vector_plot_adapter.cpp"

