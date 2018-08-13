/// HEADER
#include <csapex_core_plugins/parameter_dialog.h>

/// PROJECT
#include <csapex/param/parameter.h>
#include <csapex/param/parameter_factory.h>

/// SYSTEM
#include <QDialogButtonBox>
#include <QBoxLayout>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <limits>

ParameterDialog::ParameterDialog(const std::string& type, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f), type_(type)
{
    makeUI();
}

void ParameterDialog::makeUI()
{
    QVBoxLayout* layout = new QVBoxLayout;

    QFormLayout* form = new QFormLayout;

    name = new QLineEdit;
    name->setText("param");
    form->addRow("name", name);

    min = new QDoubleSpinBox;
    min->setMinimum(-std::numeric_limits<double>::max());
    min->setMaximum(std::numeric_limits<double>::max());
    min->setDecimals(10);
    form->addRow("minimum value", min);

    max = new QDoubleSpinBox;
    max->setMinimum(-std::numeric_limits<double>::max());
    max->setMaximum(std::numeric_limits<double>::max());
    max->setValue(1.0);
    max->setDecimals(10);
    form->addRow("maximum value", max);

    step = new QDoubleSpinBox;
    step->setMinimum(0);
    step->setMaximum(std::numeric_limits<double>::max());
    step->setValue(0.1);
    step->setDecimals(10);
    form->addRow("step_size", step);

    layout->addLayout(form);

    QDialogButtonBox* buttons = new QDialogButtonBox;
    buttons->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);


    setWindowTitle(QString("Create RangeParam<") + type_.c_str()  + ">");
    setLayout(layout);
    setModal(true);

    QObject::connect(buttons, SIGNAL(accepted()), this, SLOT(finish()));
    QObject::connect(buttons, SIGNAL(rejected()), this, SLOT(reject()));
}

csapex::param::Parameter::Ptr ParameterDialog::getParameter()
{
    return param_;
}

void ParameterDialog::finish()
{
    param_ = csapex::param::factory::declareRange<double>(name->text().toStdString(),
                min->value(), max->value(), min->value(), step->value());
    if(param_) {
        Q_EMIT accept();
    } else {
        Q_EMIT reject();
    }
}
