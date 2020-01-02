#ifndef CREATEMAPMESSAGEADAPTER_H
#define CREATEMAPMESSAGEADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/view/utility/widget_picker.h>

#include "create_map_message.h"

class QDialog;
class QProgressBar;

namespace csapex
{
class CreateMapMessageAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    CreateMapMessageAdapter(NodeFacadeImplementationPtr worker, NodeBox* parent, std::weak_ptr<CreateMapMessage> node);
    ~CreateMapMessageAdapter();

    void setupUi(QBoxLayout* layout) override;

protected:
    virtual void parameterAdded(param::ParameterPtr param);

public Q_SLOTS:
    void widgetPicked();

    void createParameter();
    void pickParameter();

    void removeParameters();

    void setNextParameterType(const QString& type);

private:
    QDialog* makeTypeDialog();

protected:
    std::weak_ptr<CreateMapMessage> wrapped_base_;

    WidgetPicker widget_picker_;

    std::string next_type_;
};

}  // namespace csapex

#endif  // CREATEMAPMESSAGEADAPTER_H
