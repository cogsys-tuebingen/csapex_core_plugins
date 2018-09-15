/// HEADER
#include <csapex_math/view/matrix_table_model.h>

/// PROJECT
#include <csapex/model/node_facade_impl.h>
#include <csapex/msg/io.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QTableView>
#include <QPainter>
#include <QGridLayout>
#include <QLabel>

using namespace csapex;

MatrixTableModel::MatrixTableModel(int rows, int cols)
    : rows_(rows), cols_(cols)
{
}

void MatrixTableModel::update(const std::vector<double>& data)
{
    beginResetModel();

    data_ = data;

    endResetModel();
}

int MatrixTableModel::rowCount(const QModelIndex &parent) const
{
    return rows_;
}

int MatrixTableModel::columnCount(const QModelIndex &parent) const
{
    return cols_;
}


bool MatrixTableModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    double& f = data_.at(index.row() * cols_ + index.column());
    bool ok = false;

    f = value.toDouble(&ok);

    if(ok) {
        QModelIndex top = createIndex(index.row(), 0);
        QModelIndex bottom = createIndex(index.row(), 5);

        dataChanged(top, bottom);
    }
    return ok;
}

QVariant MatrixTableModel::data(const QModelIndex &index, int role) const
{
    if(role != Qt::EditRole && role != Qt::DisplayRole) {
        return QVariant();
    }

    auto col = index.column();
    auto row = index.row();
    double f = data_.at(row * cols_ + col);
    return QString::number(f);
}


QVariant MatrixTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    return QVariant();
}

Qt::ItemFlags MatrixTableModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags flags = Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
    return flags;
}

std::vector<double> MatrixTableModel::getData() const
{
    return data_;
}
