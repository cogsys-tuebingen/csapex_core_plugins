#ifndef OPTIMIZER_ADAPTER_H
#define OPTIMIZER_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/view/utility/widget_picker.h>

/// COMPONENT
#include "optimizer.h"

class QDialog;
class QProgressBar;

namespace csapex
{
class OptimizerAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    OptimizerAdapter(NodeFacadeImplementationPtr worker, NodeBox* parent, std::weak_ptr<Optimizer> node);
    ~OptimizerAdapter();

    virtual void setupUi(QBoxLayout* layout);

protected:
    virtual void parameterAdded(param::ParameterPtr param);

public Q_SLOTS:
    void widgetPicked();

    void createParameter();
    void pickParameter();

    void removeParameters();

    void startOptimization();
    void stopOptimization();

    void setNextParameterType(const QString& type);

private:
    QDialog* makeTypeDialog();

protected:
    std::weak_ptr<Optimizer> wrapped_base_;

    WidgetPicker widget_picker_;

    std::string next_type_;
};

}  // namespace csapex

#endif  // OPTIMIZER_ADAPTER_H
