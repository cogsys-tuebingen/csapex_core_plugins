/// HEADER
#include "point_count_adapter.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/utility/register_node_adapter.h>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(PointCountAdapter, csapex::PointCount)

PointCountAdapter::PointCountAdapter(PointCount *node, WidgetController* widget_ctrl)
    : NodeAdapter(node, widget_ctrl), wrapped_(node)
{
    node->display_request.connect(boost::bind(&PointCountAdapter::display, this, _1));
}

void PointCountAdapter::setupUi(QBoxLayout* layout)
{
    number_ = new QLCDNumber;
    number_->setDigitCount(8);

    layout->addWidget(number_);
}

void PointCountAdapter::display(int img)
{
    number_->display(img);
}
