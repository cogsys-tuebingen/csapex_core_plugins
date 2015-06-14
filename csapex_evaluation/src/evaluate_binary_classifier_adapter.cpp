/// HEADER
#include "evaluate_binary_classifier_adapter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_node_adapter.h>

/// SYSTEM
#include <QTableView>
#include <QPainter>
#include <QVector3D>
#include <QApplication>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(EvaluateBinaryClassifierAdapter, csapex::EvaluateBinaryClassifier)

EvaluateBinaryClassifierTableModel::EvaluateBinaryClassifierTableModel() :
    rows(0)
{

}

void EvaluateBinaryClassifierTableModel::update(EvaluateBinaryClassifier::Metrics metrics)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    metrics_ = metrics;

    if(rows < (int) metrics_.size()){
        beginInsertRows(QModelIndex(), rows, metrics_.size()-1);
        rows = metrics_.size();
        endInsertRows();
    }
}

int EvaluateBinaryClassifierTableModel::rowCount(const QModelIndex &parent) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return rows;
}

int EvaluateBinaryClassifierTableModel::columnCount(const QModelIndex &parent) const
{
    return 1;
}

QVariant EvaluateBinaryClassifierTableModel::data(const QModelIndex &index, int role) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if(role == Qt::DisplayRole) {
        return metrics_[index.row()].value;
    } else if(role == Qt::ToolTipRole) {
        return QString::fromStdString(metrics_[index.row()].description);
    } else if(role == Qt::SizeHintRole) {
        QSize s(78, 0);
        return QVariant::fromValue(s);
    }


    if(role != Qt::BackgroundColorRole && role != Qt::ForegroundRole) {
        return QVariant();
    }

    const EvaluateBinaryClassifier::Metric& m = metrics_[index.row()];
    double worst = m.worst;
    double best = m.best;
    double f = (m.value - worst) / (best - worst);

    /*
     * value = worst -> f = 0
     * value = best  -> f = 1
     *
     * v = 1   worst = -1   best = 1     / 2
     *     1    =
     * v = 0   worst = -1   best = 1     / 2
     *     0.5  =
     * v = -1  worst = -1   best = 1     / 2
     *     0    =
     *
     * v = 1   worst = 0   best = 1      / 1
     *     1
     * v = 0   worst = 0   best = 1      / 1
     *     0
     *
     * v = 1   worst = 1   best = 0      / -1
     *     0
     * v = 0   worst = 1   best = 0      / -1
     *     1
     *
     */

//    static QColor min_color = QColor::fromRgb(255, 0, 0);
//    static QColor mid_color = QColor::fromRgb(255, 255, 0);
//    static QColor max_color = QColor::fromRgb(0, 255, 0);

//    int r = std::min(255, std::max(0, int(min_color.red() * (1.0-f) + max_color.red() * f)));
//    int g = std::min(255, std::max(0, int(min_color.green() * (1.0-f) + max_color.green() * f)));
//    int b = std::min(255, std::max(0, int(min_color.blue() * (1.0-f) + max_color.blue() * f)));

    static QVector3D min_color(255, 0, 0);
    static QVector3D mid_color(255, 100, 50);
    static QVector3D max_color(50, 255, 50);

    QVector3D color;
    if(f > 0.5) { // 0.5 ... 1.0
        double ff = (f - 0.5) * 2.0;
        color = mid_color * (1.0 - ff) + max_color * ff;
    } else { // 0.0 ... 0.5
        double ff = f * 2.0;
        color = min_color * (1.0 - ff) + mid_color * ff;
    }

    if (role == Qt::ForegroundRole) {
        int v = std::max(color[0], std::max(color[1], color[2])) < 100 ? 255 : 0;
        return QVariant::fromValue(QColor::fromRgb(v,v,v));
    } else {
        return QVariant::fromValue(QColor::fromRgb(color[0], color[1], color[2]));
    }

}


QVariant EvaluateBinaryClassifierTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if (role == Qt::DisplayRole) {
        return QString::fromStdString(orientation == Qt::Vertical ? metrics_[section].name : std::string("Value"));
    } else if(role == Qt::ToolTipRole) {
        return QString::fromStdString(metrics_[section].description);
    }
    return QVariant();
}




EvaluateBinaryClassifierAdapter::EvaluateBinaryClassifierAdapter(NodeWorkerWeakPtr worker, std::weak_ptr<EvaluateBinaryClassifier> node, WidgetController* widget_ctrl)
    : NodeAdapter(worker, widget_ctrl), wrapped_(node)
{
    auto n = wrapped_.lock();

    // translate to UI thread via Qt signal
    trackConnection(n->display_request.connect(std::bind(&EvaluateBinaryClassifierAdapter::displayRequest, this)));
}

void EvaluateBinaryClassifierAdapter::setupUi(QBoxLayout* layout)
{
    model_ = new EvaluateBinaryClassifierTableModel;

    table_ = new QTableView;
    table_->setModel(model_);
    table_->showGrid();

    table_->setMinimumSize(0, 0);
    table_->viewport()->setMinimumSize(0, 0);
    table_->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

    layout->addWidget(table_);

    connect(this, SIGNAL(displayRequest()), this, SLOT(display()));
}

void EvaluateBinaryClassifierAdapter::display()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    assert(QThread::currentThread() == QApplication::instance()->thread());

    model_->update(node->getMetrics());

    table_->resizeColumnsToContents();
    table_->resizeRowsToContents();
    table_->viewport()->update();
}
/// MOC
#include "moc_evaluate_binary_classifier_adapter.cpp"
