#ifndef BF_OPTIMIZER_ADAPTER_H
#define BF_OPTIMIZER_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "bf_optimizer.h"

class QDialog;
class QProgressBar;

namespace csapex {

class BFOptimizerAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    BFOptimizerAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<BFOptimizer> node);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void createParameter();
    void startOptimization();
    void stopOptimization();

    void setNextParameterType(const QString& type);

private:
    void triggerStep(int step);
    QDialog* makeTypeDialog();

protected:
    std::weak_ptr<BFOptimizer> wrapped_;

    QProgressBar* progress_;
    std::string next_type_;
};

}
#endif // BF_OPTIMIZER_ADAPTER_H
