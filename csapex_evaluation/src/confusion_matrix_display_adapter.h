#ifndef CONFUSION_MATRIX_DISPLAY_ADAPTER_H
#define CONFUSION_MATRIX_DISPLAY_ADAPTER_H


/// PROJECT
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include "confusion_matrix_display.h"

/// SYSTEM
#include <QTableView>
#include <QStyledItemDelegate>

namespace csapex {

class ConfusionMatrixTableModel : public QAbstractTableModel
{
public:
    ConfusionMatrixTableModel();

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;


    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    virtual QVariant headerData(int section, Qt::Orientation orientation,
                                int role = Qt::DisplayRole) const;

    void update(const ConfusionMatrix &confusion_matrix);

private:
    ConfusionMatrix confusion_;

    std::vector<int> sum;
    int dim;
};

class ConfusionMatrixDisplayAdapter : public QObject, public NodeAdapter
{
    Q_OBJECT

public:
    ConfusionMatrixDisplayAdapter(NodeWorkerWeakPtr worker, ConfusionMatrixDisplay *node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    ConfusionMatrixDisplay* wrapped_;

private:
    ConfusionMatrixTableModel* model_;
    QTableView* table_;
};

}
#endif // CONFUSION_MATRIX_DISPLAY_ADAPTER_H
