/// HEADER
#include "confidence_matrix_display_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_node_adapter.h>

/// SYSTEM
#include <QTableView>
#include <QPainter>
#include <QGridLayout>
#include <QLabel>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(ConfidenceMatrixDisplayAdapter, csapex::ConfidenceMatrixDisplay)

ConfidenceMatrixTableModel::ConfidenceMatrixTableModel()
    : dim(0)
{

}

void ConfidenceMatrixTableModel::update(const ConfidenceMatrix& confidence)
{

    int new_dim = confidence.classes.size();
    if(dim != new_dim) {
        beginInsertRows(QModelIndex(), dim, new_dim - 1);
        beginInsertColumns(QModelIndex(), dim, new_dim - 1);

        dim = new_dim;

        endInsertRows();
        endInsertColumns();
    }

    confidence_ = confidence;
}

int ConfidenceMatrixTableModel::rowCount(const QModelIndex &parent) const
{
    return dim;
}

int ConfidenceMatrixTableModel::columnCount(const QModelIndex &parent) const
{
    return dim;
}

QVariant ConfidenceMatrixTableModel::data(const QModelIndex &index, int role) const
{
    if(role != Qt::ForegroundRole && role != Qt::BackgroundColorRole && role != Qt::DisplayRole) {
        return QVariant();
    }

    auto actual = index.column();
    auto prediction = index.row();
    double f = confidence_.confidences.at(std::make_pair(actual, prediction));
    if(role == Qt::DisplayRole) {
        return f;
    }

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


QVariant ConfidenceMatrixTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();
    return confidence_.classes.at(section);
}




ConfidenceMatrixDisplayAdapter::ConfidenceMatrixDisplayAdapter(NodeWorker *worker, ConfidenceMatrixDisplay *node, WidgetController* widget_ctrl)
    : NodeAdapter(worker, widget_ctrl), wrapped_(node)
{
    // translate to UI thread via Qt signal
    node->display_request.connect(std::bind(&ConfidenceMatrixDisplayAdapter::displayRequest, this));
}

void ConfidenceMatrixDisplayAdapter::setupUi(QBoxLayout* layout)
{
    model_ = new ConfidenceMatrixTableModel;

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

void ConfidenceMatrixDisplayAdapter::display()
{
    model_->update(wrapped_->getConfidenceMatrix());

    table_->resizeColumnsToContents();
    table_->resizeRowsToContents();
    table_->viewport()->update();
}
/// MOC
#include "moc_confidence_matrix_display_adapter.cpp"
