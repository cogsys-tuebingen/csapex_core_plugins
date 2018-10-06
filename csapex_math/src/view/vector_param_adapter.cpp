/// HEADER
#include <csapex_math/view/vector_param_adapter.h>

/// PROJECT
#include <csapex/command/update_parameter.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/utility/register_param_adapter.h>
#include <csapex_math/view/matrix_dialog.h>

/// SYSTEM
#include <QBoxLayout>
#include <QDial>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <iostream>

using namespace csapex;

CSAPEX_REGISTER_PARAM_ADAPTER(csapex, VectorParameterAdapter, csapex::param::LinearVectorParameter)

VectorParameterAdapter::VectorParameterAdapter(param::LinearVectorParameter::Ptr p) : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), vector_p_(p)
{
    makeString();
}

QWidget* VectorParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QPushButton> btn = new QPushButton(string_);

    btn->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(btn.data(), &QPushButton::customContextMenuRequested, [=](const QPoint& point) { customContextMenuRequested(btn, point); });

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    layout->addLayout(QtHelper::wrap(display_name, sub, context_handler));

    // ui callback
    QObject::connect(btn.data(), &QPushButton::pressed, [this, btn]() {
        if (!vector_p_ || !btn) {
            return;
        }

        MatrixDialog diag(vector_p_->getValue().size(), 1, vector_p_->getValue().getData());
        diag.setModal(true);

        if (!diag.exec()) {
            return;
        }

        set(diag.getData());
    });

    // model change -> ui
    connectInGuiThread(p_->parameter_changed, [this, btn](param::Parameter*) {
        if (!vector_p_ || !btn) {
            return;
        }
        makeString();
        btn->setText(string_);
    });

    return btn;
}

void VectorParameterAdapter::setupContextMenu(ParameterContextMenu* context_handler)
{
    context_handler->addAction(new QAction("set all 0", context_handler), [this]() {
        std::vector<double> v(vector_p_->getValue().size(), 0);
        set(v);
    });
    context_handler->addAction(new QAction("set all 1", context_handler), [this]() {
        std::vector<double> v(vector_p_->getValue().size(), 1);
        set(v);
    });
}

void VectorParameterAdapter::makeString()
{
    QString s = "(";

    std::size_t n = vector_p_->getValue().size();
    for (std::size_t i = 0; i < n; ++i) {
        if (i > 0) {
            s += " ";
        }
        s += QString::number(vector_p_->getValue()(i));
    }

    s += ")";

    string_ = s;
}

void VectorParameterAdapter::set(const std::vector<double>& data)
{
    auto p = vector_p_->cloneAs<param::LinearVectorParameter>();
    p->set(math::linear::Vector(data));
    command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(vector_p_->getUUID().getAbsoluteUUID(), *p);
    executeCommand(update_parameter);
}
