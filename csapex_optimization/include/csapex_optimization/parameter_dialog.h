#ifndef PARAMETER_DIALOG_H
#define PARAMETER_DIALOG_H

/// PROJECT
#include <utils_param/param_fwd.h>

/// SYSTEM
#include <QDialog>

class QLineEdit;
class QDoubleSpinBox;

class ParameterDialog : public QDialog
{
    Q_OBJECT

public:
    ParameterDialog(const std::string &type, QWidget *parent = 0, Qt::WindowFlags f = 0);

    param::ParameterPtr getParameter();

private Q_SLOTS:
    void finish();

private:
    void makeUI();

private:
    std::string type_;

    QLineEdit* name;
    QDoubleSpinBox* min;
    QDoubleSpinBox* max;
    QDoubleSpinBox* step;

    param::ParameterPtr param_;
};

#endif // PARAMETER_DIALOG_H
