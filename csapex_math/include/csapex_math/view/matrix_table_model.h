#ifndef MATRIX_TABLE_MODEL_H
#define MATRIX_TABLE_MODEL_H

/// SYSTEM
#include <QStyledItemDelegate>
#include <QTableView>

namespace csapex
{
class MatrixTableModel : public QAbstractTableModel
{
public:
    MatrixTableModel(int rows, int cols);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;

    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    Qt::ItemFlags flags(const QModelIndex& index) const override;

    void update(const std::vector<double>& data);

    std::vector<double> getData() const;

private:
    std::vector<double> data_;

    int rows_;
    int cols_;
};

}  // namespace csapex

#endif  // MATRIX_TABLE_MODEL_H
