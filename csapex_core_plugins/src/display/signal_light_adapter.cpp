/// HEADER
#include "signal_light_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QBoxLayout>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(SignalLightAdapter, csapex::SignalLight)


SignalLightAdapter::SignalLightAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<SignalLight> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node)
{
    auto n = wrapped_.lock();

    // translate to UI thread via Qt signal
    trackConnection(n->display_request.connect(std::bind(&SignalLightAdapter::displayRequest, this, std::placeholders::_1)));
}

Memento::Ptr SignalLightAdapter::getState() const
{
    return std::shared_ptr<State>(new State(state));
}

void SignalLightAdapter::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<State> m = std::dynamic_pointer_cast<State> (memento);
    apex_assert(m.get());

    state = *m;

    light_->setFixedSize(QSize(state.width, state.height));
}

void SignalLightAdapter::setupUi(QBoxLayout* layout)
{
    DefaultNodeAdapter::setupUi(layout);

    light_ = new SignalLightWidget;
    layout->addWidget(light_);
    layout->setMargin(0);

    layout->setStretch(1, 100);

    setManualResize(true);

    light_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

    light_->gren()->setEnabled(true);
    light_->yellow()->setEnabled(false);
    light_->red()->setEnabled(false);


    connect(this, SIGNAL(displayRequest(int)), this, SLOT(display(int)));
}

void SignalLightAdapter::display(int state)
{
    light_->gren()->setEnabled(state == 0);
    light_->yellow()->setEnabled(state == 1);
    light_->red()->setEnabled(state == 2);
}

bool SignalLightAdapter::isResizable() const
{
    return true;
}
void SignalLightAdapter::setManualResize(bool manual)
{
    if(manual) {
        light_->setMinimumSize(20, 60);
        light_->setMaximumSize(200, 600);
    } else {
        QSize s = light_->size();
        light_->setFixedSize(s);
        state.width = s.width();
        state.height = s.height();
    }
}

/// MOC
#include "../moc_signal_light_adapter.cpp"

