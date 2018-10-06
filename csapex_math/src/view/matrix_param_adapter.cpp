/// HEADER
#include <csapex_math/view/matrix_param_adapter.h>

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

CSAPEX_REGISTER_PARAM_ADAPTER(csapex, MatrixParameterAdapter, csapex::param::LinearMatrixParameter)

MatrixParameterAdapter::MatrixParameterAdapter(param::LinearMatrixParameter::Ptr p) : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), matrix_p_(p)
{
    makeString();
}

QWidget* MatrixParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QPushButton> btn = new QPushButton(string_);

    btn->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(btn.data(), &QPushButton::customContextMenuRequested, [=](const QPoint& point) { customContextMenuRequested(btn, point); });

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    layout->addLayout(QtHelper::wrap(display_name, sub, context_handler));

    // ui callback
    QObject::connect(btn.data(), &QPushButton::pressed, [this, btn]() {
        if (!matrix_p_ || !btn) {
            return;
        }

        math::linear::Matrix matrix = matrix_p_->getValue();
        MatrixDialog diag(matrix.rows(), matrix.cols(), matrix.getData());
        diag.setModal(true);

        if (!diag.exec()) {
            return;
        }

        set(diag.getData());
    });

    // model change -> ui
    connectInGuiThread(p_->parameter_changed, [this, btn](param::Parameter*) {
        if (!matrix_p_ || !btn) {
            return;
        }
        makeString();
        btn->setText(string_);
    });

    return btn;
}

void MatrixParameterAdapter::setupContextMenu(ParameterContextMenu* context_handler)
{
    context_handler->addAction(new QAction("set all 0", context_handler), [this]() {
        std::vector<double> v(matrix_p_->getValue().size(), 0);
        set(v);
    });
    context_handler->addAction(new QAction("set all 1", context_handler), [this]() {
        std::vector<double> v(matrix_p_->getValue().size(), 1);
        set(v);
    });
    context_handler->addAction(new QAction("set to identity", context_handler), [this]() {
        const math::linear::Matrix& matrix = matrix_p_->getValue();
        std::vector<double> v(matrix.size(), 0);
        for(int i = 0, n = std::min(matrix.rows(), matrix.cols()); i < n; ++i) {
            v.at(i * matrix.cols() + i) = 1;
        }
        set(v);
    });
}

void MatrixParameterAdapter::makeString()
{
    QString s = "";

    math::linear::Matrix matrix = matrix_p_->getValue();
    int rows = matrix.rows();
    int cols = matrix.cols();
    for (int row = 0; row < rows; ++row) {
        if (row > 0) {
            s += "\n";
        }
        for (int col = 0; col < cols; ++col) {
            s += QString::number(matrix(row, col)) + " ";
        }
    }

    s += "";

    string_ = s;
}

void MatrixParameterAdapter::set(const std::vector<double>& data)
{
    auto p = matrix_p_->cloneAs<param::LinearMatrixParameter>();
    p->set(math::linear::Matrix(p->getValue().rows(), p->getValue().cols(), data));
    command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(matrix_p_->getUUID().getAbsoluteUUID(), *p);
    executeCommand(update_parameter);
}
