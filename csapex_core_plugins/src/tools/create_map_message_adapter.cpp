
/// HEADER
#include "create_map_message_adapter.h"

/// PROJECT
#include <csapex/model/node_facade_impl.h>
#include <csapex_core_plugins/parameter_dialog.h>

/// PROJECT
#include <csapex/command/add_connection.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/meta.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node_handle.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/node/box.h>
#include <csapex/view/utility/register_node_adapter.h>

/// SYSTEM
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QProgressBar>
#include <QPushButton>
#include <iostream>

using namespace csapex;

CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(CreateMapMessageAdapter, csapex::CreateMapMessage)

CreateMapMessageAdapter::CreateMapMessageAdapter(NodeFacadeImplementationPtr worker, NodeBox* parent, std::weak_ptr<CreateMapMessage> node) : DefaultNodeAdapter(worker, parent), wrapped_base_(node)
{
    QObject::connect(&widget_picker_, SIGNAL(widgetPicked()), this, SLOT(widgetPicked()));
    auto n = wrapped_base_.lock();
}

CreateMapMessageAdapter::~CreateMapMessageAdapter()
{
}

void CreateMapMessageAdapter::setupUi(QBoxLayout* layout)
{
    DefaultNodeAdapter::setupUi(layout);

    QPushButton* btn_add_param = new QPushButton("Create Parameter");
    layout->addWidget(btn_add_param);
    QObject::connect(btn_add_param, SIGNAL(clicked()), this, SLOT(createParameter()));

    QPushButton* btn_pick_param = new QPushButton("Pick Parameter");
    layout->addWidget(btn_pick_param);
    QObject::connect(btn_pick_param, SIGNAL(clicked()), this, SLOT(pickParameter()));

    QPushButton* btn_remove_param = new QPushButton("Remove Parameters");
    layout->addWidget(btn_remove_param);
    QObject::connect(btn_remove_param, SIGNAL(clicked()), this, SLOT(removeParameters()));
}

void CreateMapMessageAdapter::parameterAdded(param::ParameterPtr param)
{
}

void CreateMapMessageAdapter::widgetPicked()
{
    auto node = wrapped_base_.lock();
    if (!node) {
        return;
    }

    QWidget* widget = widget_picker_.getWidget();
    if (widget) {
        if (widget != nullptr) {
            std::cerr << "picked widget " << widget->metaObject()->className() << std::endl;
        }

        QVariant var = widget->property("parameter");
        if (!var.isNull()) {
            csapex::param::Parameter* connected_parameter = static_cast<csapex::param::Parameter*>(var.value<void*>());

            if (connected_parameter != nullptr) {
                node->ainfo << "picked parameter " << connected_parameter->name() << " with UUID " << connected_parameter->getUUID() << std::endl;

                std::string label = connected_parameter->name();
                Input* i = node->createVariadicInput(makeEmpty<connection_types::AnyMessage>(), label, false);

                UUID input = i->getUUID();
                UUID output = UUIDProvider::makeDerivedUUID_forced(connected_parameter->getUUID().parentUUID(), std::string("out_") + connected_parameter->name());

                if (!connected_parameter->isInteractive()) {
                    connected_parameter->setInteractive(true);
                }

                GraphFacade* facade = parent_->getGraphView()->getGraphFacade();

                if (!facade->isConnected(input, output)) {
                    AUUID parent_uuid = facade->getAbsoluteUUID();
                    executeCommand(std::make_shared<command::AddConnection>(parent_uuid, output, input, false));

                } else {
                    node->getNodeHandle()->setError("the selected parameter is already connected.");
                }

            } else {
                node->getNodeHandle()->setWarning("No Parameter selected.");
            }
        } else {
            node->getNodeHandle()->setWarning("The widget has no parameter property.");
        }
    } else {
        node->getNodeHandle()->setWarning("No widget selected.");
    }
}

QDialog* CreateMapMessageAdapter::makeTypeDialog()
{
    QVBoxLayout* layout = new QVBoxLayout;

    QFormLayout* form = new QFormLayout;

    QComboBox* type = new QComboBox;
    type->addItem("int");
    type->addItem("double");
    form->addRow("Type", type);

    QObject::connect(type, SIGNAL(currentIndexChanged(QString)), this, SLOT(setNextParameterType(QString)));
    next_type_ = type->currentText().toStdString();

    layout->addLayout(form);

    QDialogButtonBox* buttons = new QDialogButtonBox;
    buttons->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    QDialog* dialog = new QDialog;
    dialog->setWindowTitle("Create Parameter");
    dialog->setLayout(layout);
    dialog->setModal(true);

    QObject::connect(buttons, SIGNAL(accepted()), dialog, SLOT(accept()));
    QObject::connect(buttons, SIGNAL(rejected()), dialog, SLOT(reject()));

    return dialog;
}

void CreateMapMessageAdapter::setNextParameterType(const QString& type)
{
    next_type_ = type.toStdString();
}

void CreateMapMessageAdapter::pickParameter()
{
    auto designer = parent_->getGraphView()->designerScene();
    if (designer) {
        widget_picker_.startPicking(designer);
    }
}

void CreateMapMessageAdapter::createParameter()
{
    std::cerr << "parmeters currently unused" << std::endl;
    auto node = wrapped_base_.lock();
    if (!node) {
        return;
    }
    QDialog* type_dialog = makeTypeDialog();
    if (type_dialog->exec() == QDialog::Accepted) {
        ParameterDialog diag(next_type_);
        if (diag.exec() == QDialog::Accepted) {
            csapex::param::Parameter::Ptr param = diag.getParameter();
            node->addPersistentParameter(param);

            parameterAdded(param);

            node->reset();
        }
    }
}

void CreateMapMessageAdapter::removeParameters()
{
    auto node = wrapped_base_.lock();
    if (!node) {
        return;
    }

    GraphFacade* facade = parent_->getGraphView()->getGraphFacade();

    command::Meta::Ptr cmd(new command::Meta(facade->getAbsoluteUUID(), "remove parameter", true));
    NodeHandle* nh = node->getNodeHandle();

    CommandFactory factory(facade);

    for (param::ParameterPtr p : node->getPersistentParameters()) {
        if (OutputPtr out = nh->getParameterOutput(p->name()).lock()) {
            cmd->add(factory.removeAllConnectionsCmd(out));
        }
        if (InputPtr in = nh->getParameterInput(p->name()).lock()) {
            cmd->add(factory.removeAllConnectionsCmd(in));
        }
    }

    executeCommand(cmd);

    node->removePersistentParameters();
}

/// MOC
#include "moc_create_map_message_adapter.cpp"
