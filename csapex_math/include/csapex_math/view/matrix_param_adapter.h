#ifndef ANGLE_PARAM_ADAPTER_H
#define ANGLE_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex_math/param/linear_matrix_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class CSAPEX_QT_EXPORT MatrixParameterAdapter : public ParameterAdapter
{
public:
    MatrixParameterAdapter(param::LinearMatrixParameter::Ptr p);

    QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;
    void setupContextMenu(ParameterContextMenu* context_handler) override;

private:
    void set(const std::vector<double>& angle);
    void makeString();

private:
    param::LinearMatrixParameter::Ptr matrix_p_;
    QString string_;
};

}  // namespace csapex

#endif  // ANGLE_PARAM_ADAPTER_H
