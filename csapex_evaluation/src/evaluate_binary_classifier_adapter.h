#ifndef EVALUATE_BINARY_CLASSIFIER_ADAPTER_H
#define EVALUATE_BINARY_CLASSIFIER_ADAPTER_H

/// PROJECT
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include "evaluate_binary_classifier.h"
#include <csapex_evaluation/confusion_matrix.h>

/// SYSTEM
#include <QTableView>
#include <QStyledItemDelegate>

namespace csapex {

class EvaluateBinaryClassifierTableModel : public QAbstractTableModel
{
public:
    EvaluateBinaryClassifierTableModel();

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;


    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    virtual QVariant headerData(int section, Qt::Orientation orientation,
                                int role = Qt::DisplayRole) const;

    void update(EvaluateBinaryClassifier::Metrics metrics);

private:
    mutable std::recursive_mutex mutex_;
    EvaluateBinaryClassifier::Metrics metrics_;
    int rows;
};

class EvaluateBinaryClassifierAdapter : public QObject, public NodeAdapter
{
    Q_OBJECT

public:
    EvaluateBinaryClassifierAdapter(NodeWorkerWeakPtr worker, EvaluateBinaryClassifier* node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    EvaluateBinaryClassifier* wrapped_;

private:
    EvaluateBinaryClassifierTableModel* model_;
    QTableView* table_;
};

}
#endif // EVALUATE_BINARY_CLASSIFIER_ADAPTER_H
