/// HEADER
#include <csapex_math/view/matrix_dialog.h>

/// COMPONENT
#include <csapex/view/utility/html_delegate.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/model/graph_facade.h>

/// SYSTEM
#include <QLabel>
#include <QRadioButton>
#include <QButtonGroup>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <QVBoxLayout>

using namespace csapex;

MatrixDialog::MatrixDialog(int rows, int cols, const std::vector<double>& data, QWidget* parent, Qt::WindowFlags f) : QDialog(parent, f), data_(data)
{
    setWindowIcon(QIcon(":/image.png"));
    setWindowTitle("Matrix Editor");

    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;

    model_ = new MatrixTableModel(rows, cols);
    model_->update(data_);

    table_ = new QTableView;
    table_->setModel(model_);
    table_->showGrid();

    table_->setMinimumSize(0, 0);
    table_->viewport()->setMinimumSize(0, 0);
    table_->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

    layout->addWidget(table_);

    button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Reset | QDialogButtonBox::Cancel);

    connect(button_box_, SIGNAL(clicked(QAbstractButton*)), this, SLOT(handle(QAbstractButton*)));

    layout->addWidget(button_box_);

    setLayout(layout);
}

void MatrixDialog::handle(QAbstractButton* button)
{
    switch (button_box_->buttonRole(button)) {
        case QDialogButtonBox::ButtonRole::ResetRole:
            model_->update(data_);
            break;
        case QDialogButtonBox::ButtonRole::AcceptRole:
            data_ = model_->getData();
            accept();
            break;
        case QDialogButtonBox::ButtonRole::RejectRole:
            reject();
            break;
        default:
            reject();
            break;
    }
}

std::vector<double> MatrixDialog::getData() const
{
    return data_;
}

/// MOC
#include "../../include/csapex_math/view/moc_matrix_dialog.cpp"
