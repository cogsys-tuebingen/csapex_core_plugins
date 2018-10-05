/// HEADER
#include "signal_light_adapter.h"

/// PROJECT
#include <csapex/io/raw_message.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QBoxLayout>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(SignalLightAdapter, csapex::SignalLight)

SignalLightAdapter::SignalLightAdapter(NodeFacadePtr node, NodeBox* parent) : ResizableNodeAdapter(node, parent)
{
    observe(node->raw_data_connection, [this](StreamableConstPtr msg) {
        if (std::shared_ptr<RawMessage const> raw = std::dynamic_pointer_cast<RawMessage const>(msg)) {
            displayRequest(raw->getData().at(0));
        }
    });
}

SignalLightAdapter::~SignalLightAdapter()
{
    stopObserving();
}

void SignalLightAdapter::resize(const QSize& size)
{
    light_->setFixedSize(size);
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

    connect(this, &SignalLightAdapter::displayRequest, this, &SignalLightAdapter::display);

    ResizableNodeAdapter::setupUi(layout);
}

void SignalLightAdapter::display(int state)
{
    light_->gren()->setEnabled(state == 0);
    light_->yellow()->setEnabled(state == 1);
    light_->red()->setEnabled(state == 2);
}

void SignalLightAdapter::setManualResize(bool manual)
{
    if (manual) {
        light_->setMinimumSize(20, 60);
        light_->setMaximumSize(200, 600);
    } else {
        QSize s = light_->size();
        light_->setFixedSize(s);
        setSize(s.width(), s.height());
    }
}

/// MOC
#include "moc_signal_light_adapter.cpp"
