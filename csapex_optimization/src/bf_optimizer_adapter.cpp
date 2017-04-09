/// HEADER
#include "bf_optimizer_adapter.h"

/// PROJECT
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QProgressBar>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(BFOptimizerAdapter, csapex::BFOptimizer)


BFOptimizerAdapter::BFOptimizerAdapter(NodeFacadeWeakPtr worker, NodeBox* parent, std::weak_ptr<BFOptimizer> node)
    : OptimizerAdapter(worker, parent, node), wrapped_(node)
{
    auto n = wrapped_.lock();
    trackConnection(n->step.connect(std::bind(&BFOptimizerAdapter::triggerStep, this, std::placeholders::_1)));
}

BFOptimizerAdapter::~BFOptimizerAdapter()
{
}

void BFOptimizerAdapter::setupUi(QBoxLayout* layout)
{
    OptimizerAdapter::setupUi(layout);
    progress_ = new QProgressBar;
    layout->addWidget(progress_);
}

void BFOptimizerAdapter::triggerStep(int s)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    progress_->setValue(s);
    progress_->setMaximum(node->stepsNecessary());
}

void BFOptimizerAdapter::parameterAdded(param::ParameterPtr p)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    progress_->setMaximum(node->stepsNecessary());
}

/// MOC
#include "moc_bf_optimizer_adapter.cpp"
