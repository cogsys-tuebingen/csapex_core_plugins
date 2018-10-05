#ifndef MATRIX_DIALOG_H
#define MATRIX_DIALOG_H

/// COMPONENT
#include <csapex_math/view/matrix_table_model.h>
#include <csapex_qt_export.h>

/// PROJECT
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <QDialog>

class QGraphicsView;
class QRadioButton;
class QDialogButtonBox;
class QAbstractButton;

namespace csapex
{
class CSAPEX_QT_EXPORT MatrixDialog : public QDialog
{
    Q_OBJECT

public:
    MatrixDialog(int rows, int cols, const std::vector<double>& data, QWidget* parent = 0, Qt::WindowFlags f = 0);

    std::vector<double> getData() const;

private Q_SLOTS:
    void handle(QAbstractButton* button);

private:
    MatrixTableModel* model_;
    QTableView* table_;
    QDialogButtonBox* button_box_;

    std::vector<double> data_;
};

}  // namespace csapex

#endif  // MATRIX_DIALOG_H
