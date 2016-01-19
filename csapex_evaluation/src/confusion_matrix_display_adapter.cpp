/// HEADER
#include "confusion_matrix_display_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QTableView>
#include <QPainter>
#include <QGridLayout>
#include <QLabel>
#include <QApplication>

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
            sum[col] += confusion.histogram.at(std::make_pair(confusion.classes[row], confusion.classes[col]));
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

    auto actual = confusion_.classes[index.column()];
    auto prediction = confusion_.classes[index.row()];
    int entry = confusion_.histogram.at(std::make_pair(actual, prediction));
    if(role == Qt::DisplayRole) {
        return entry;
    }

    double f = entry / double(sum[actual]);

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




ConfusionMatrixDisplayAdapter::ConfusionMatrixDisplayAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<ConfusionMatrixDisplay> node)
    : NodeAdapter(worker, parent), wrapped_(node)
{
    auto n = wrapped_.lock();
    // translate to UI thread via Qt signal
    trackConnection(n ->display_request.connect(std::bind(&ConfusionMatrixDisplayAdapter::displayRequest, this)));
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

    QGridLayout* grid = new QGridLayout;
    layout->addLayout(grid);

    auto actual = new QLabel("actual");

    grid->addWidget(actual, 0, 1);
    grid->addWidget(new QLabel("predicted"), 1, 0);

    grid->addWidget(table_, 1, 1);


    connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

void ConfusionMatrixDisplayAdapter::display()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    model_->update(node->getConfusionMatrix());

    table_->resizeColumnsToContents();
    table_->resizeRowsToContents();
    table_->viewport()->update();
}
/// MOC
#include "moc_confusion_matrix_display_adapter.cpp"
