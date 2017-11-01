/// HEADER
#include "point_count_adapter.h"

/// PROJECT
#include <csapex/model/node_facade_local.h>
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

using namespace csapex;

CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(PointCountAdapter, csapex::PointCount)

PointCountAdapter::PointCountAdapter(NodeFacadeLocalPtr worker, NodeBox* parent, std::weak_ptr<PointCount> node)
    : DefaultNodeAdapter(worker, parent), wrapped_(node)
{
    auto n = wrapped_.lock();

    observe(n->display_request, this, &PointCountAdapter::display);
}

void PointCountAdapter::setupUi(QBoxLayout* layout)
{
    number_ = new QLCDNumber;
    number_->setDigitCount(8);

    layout->addWidget(number_);

    QObject::connect(this, SIGNAL(displayRequest(int)), number_, SLOT(display(int)));

    DefaultNodeAdapter::setupUi(layout);
}

void PointCountAdapter::display(int img)
{
    Q_EMIT displayRequest(img);
}
/// MOC
#include "moc_point_count_adapter.cpp"
