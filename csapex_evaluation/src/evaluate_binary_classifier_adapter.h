#ifndef EVALUATE_BINARY_CLASSIFIER_ADAPTER_H
#define EVALUATE_BINARY_CLASSIFIER_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

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

class EvaluateBinaryClassifierAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    EvaluateBinaryClassifierAdapter(NodeFacadeLocalPtr worker, NodeBox* parent, std::weak_ptr<EvaluateBinaryClassifier> node);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display();

Q_SIGNALS:
    void displayRequest();

protected:
    std::weak_ptr<EvaluateBinaryClassifier> wrapped_;

private:
    EvaluateBinaryClassifierTableModel* model_;
    QTableView* table_;
};

}
#endif // EVALUATE_BINARY_CLASSIFIER_ADAPTER_H
