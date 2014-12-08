#ifndef SCAN_LABELER_ADAPTER_H
#define SCAN_LABELER_ADAPTER_H

/// PROJECT
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include "scan_labeler.h"
#include <csapex_scan_2d/labeled_scan_message.h>

/// SYSTEM
#include <QGraphicsView>

namespace csapex {

class ScanLabelerAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    ScanLabelerAdapter(NodeWorker *worker, ScanLabeler *node, WidgetController *widget_ctrl);

    virtual Memento::Ptr getState() const;
    virtual void setParameterState(Memento::Ptr memento);

    virtual void setupUi(QBoxLayout* layout);

    void updateLabel(int label);

public Q_SLOTS:
    void display(lib_laser_processing::Scan* img);
    void submit();
    void labelSelected();

Q_SIGNALS:
    void displayRequest(lib_laser_processing::Scan* img);
    void submitRequest();

protected:
    bool eventFilter(QObject* o, QEvent* e);
    void labelSelected(int label);

    ScanLabeler* wrapped_;

    struct State : public Memento {
        int width;
        int height;

        State()
            : width(300), height(300)
        {}

        virtual void writeYaml(YAML::Node& out) const {
            out["width"] = width;
            out["height"] = height;
        }
        virtual void readYaml(const YAML::Node& node) {
            if(node["width"].IsDefined())
                width = node["width"].as<int>();
            if(node["height"].IsDefined())
                height = node["height"].as<int>();
        }
    };

private:
    static const double SCALE = 100.0;

private:
    connection_types::LabeledScanMessage::Ptr result_;

    QSize last_size_;
    State state;

    QGraphicsPixmapItem* pixmap_;

    QGraphicsView* view_;

    bool resize_down_;
    bool move_down_;
    QPoint last_pos_;
};

}

#endif // SCAN_LABELER_ADAPTER_H
