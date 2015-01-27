/// HEADER
#include "confusion_matrix_display_adapter.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_node_adapter.h>

/// SYSTEM
#include <QTableView>
#include <QPainter>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(ConfusionMatrixDisplayAdapter, csapex::ConfusionMatrixDisplay)

ConfusionMatrixTableModel::ConfusionMatrixTableModel()
    : dim(0)
{

}

void ConfusionMatrixTableModel::update(const ConfusionMatrix& confusion)
{

    int new_dim = confusion_.classes.size();
    if(dim != new_dim) {
        beginInsertRows(QModelIndex(), dim, new_dim - 1);
        beginInsertColumns(QModelIndex(), dim, new_dim - 1);

        dim = new_dim;

        endInsertRows();
        endInsertColumns();
    }

    confusion_ = confusion;

    sum.resize(dim);

    for(int col = 0; col < dim; ++col) {
        sum[col] = 0;
        for(int row = 0; row < dim; ++row) {
            sum[col] += confusion.histogram.at(std::make_pair(row, col));
        }
    }
}

int ConfusionMatrixTableModel::rowCount(const QModelIndex &parent) const
{
    return dim;
}

int ConfusionMatrixTableModel::columnCount(const QModelIndex &parent) const
{
    return dim;
}

QVariant ConfusionMatrixTableModel::data(const QModelIndex &index, int role) const
{
    if(role != Qt::ForegroundRole && role != Qt::BackgroundColorRole && role != Qt::DisplayRole) {
        return QVariant();
    }

    int entry = confusion_.histogram.at(std::make_pair(index.row(), index.column()));
    if(role == Qt::DisplayRole) {
        return entry;
    }

    double f = entry / double(sum[index.column()]);

    static QColor min_color = QColor::fromRgb(255, 255, 255);
    static QColor max_color = QColor::fromRgb(0, 0, 0);

    int grey = std::min(255, std::max(0, int(min_color.red() * (1.0-f) + max_color.red() * f)));

    if (role == Qt::ForegroundRole) {
        int v = grey < 100 ? 255 : 0;
        return QVariant::fromValue(QColor::fromRgb(v,v,v));
    } else {
        return QVariant::fromValue(QColor::fromRgb(grey,grey,grey));
    }
}


QVariant ConfusionMatrixTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();
    return confusion_.classes.at(section);
}




ConfusionMatrixDisplayAdapter::ConfusionMatrixDisplayAdapter(NodeWorker *worker, ConfusionMatrixDisplay *node, WidgetController* widget_ctrl)
    : NodeAdapter(worker, widget_ctrl), wrapped_(node)
{
    // translate to UI thread via Qt signal
    node->display_request.connect(std::bind(&ConfusionMatrixDisplayAdapter::displayRequest, this));
}

void ConfusionMatrixDisplayAdapter::setupUi(QBoxLayout* layout)
{
    model_ = new ConfusionMatrixTableModel;

    table_ = new QTableView;
    table_->setModel(model_);
    table_->showGrid();

    table_->setMinimumSize(0, 0);
    table_->viewport()->setMinimumSize(0, 0);
    table_->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

    layout->addWidget(table_);

    connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

void ConfusionMatrixDisplayAdapter::display()
{
    model_->update(wrapped_->getConfusionMatrix());

    table_->resizeColumnsToContents();
    table_->resizeRowsToContents();
    table_->viewport()->update();
}
/// MOC
#include "moc_confusion_matrix_display_adapter.cpp"
