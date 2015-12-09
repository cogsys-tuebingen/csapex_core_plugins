#ifndef CONFIDENCE_MATRIX_DISPLAY_ADAPTER_H
#define CONFIDENCE_MATRIX_DISPLAY_ADAPTER_H


/// PROJECT
#include <csapex/view/node/node_adapter.h>

/// COMPONENT
#include "confidence_matrix_display.h"

/// SYSTEM
#include <QTableView>
#include <QStyledItemDelegate>

namespace csapex {

class ConfidenceMatrixTableModel : public QAbstractTableModel
{
public:
    ConfidenceMatrixTableModel();

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;


    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    virtual QVariant headerData(int section, Qt::Orientation orientation,
                                int role = Qt::DisplayRole) const;

    void update(const ConfidenceMatrix &confidence);

private:
    ConfidenceMatrix confidence_;
    int dim;
};

class ConfidenceMatrixDisplayAdapter : public QObject, public NodeAdapter
{
    Q_OBJECT

public:
    ConfidenceMatrixDisplayAdapter(NodeHandleWeakPtr worker, std::weak_ptr<ConfidenceMatrixDisplay> node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    std::weak_ptr<ConfidenceMatrixDisplay>    wrapped_;

private:
    ConfidenceMatrixTableModel* model_;
    QTableView*                 table_;
};

}
#endif // CONFIDENCE_MATRIX_DISPLAY_ADAPTER_H
