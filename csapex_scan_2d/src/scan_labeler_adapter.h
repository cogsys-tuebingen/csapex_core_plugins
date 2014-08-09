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
    ScanLabelerAdapter(ScanLabeler *node, WidgetController *widget_ctrl);

    virtual Memento::Ptr getState() const;
    virtual void setParameterState(Memento::Ptr memento);

    virtual void setupUi(QBoxLayout* layout);

    void setLabel(int label);
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

        virtual void writeYaml(YAML::Emitter& out) const {
            out << YAML::Key << "width" << YAML::Value << width;
            out << YAML::Key << "height" << YAML::Value << height;
        }
        virtual void readYaml(const YAML::Node& node) {
            node["width"] >> width;
            node["height"] >> height;
        }
    };


private:
    connection_types::LabeledScanMessage::Ptr result_;

    QSize last_size_;
    State state;

    QGraphicsPixmapItem* pixmap_;

    QGraphicsView* view_;

    QImage empty;
    QPainter painter;

    int label_;
    bool down_;
    QPoint last_pos_;
};

}

#endif // SCAN_LABELER_ADAPTER_H
