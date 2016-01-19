#ifndef SCAN_LABELER_ADAPTER_H
#define SCAN_LABELER_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "scan_labeler.h"
#include <csapex_scan_2d/labeled_scan_message.h>

/// SYSTEM
#include <QGraphicsView>
#include <yaml-cpp/yaml.h>

namespace csapex {

class ScanLabelerAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    ScanLabelerAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<ScanLabeler> node);

    virtual Memento::Ptr getState() const;
    virtual void setParameterState(Memento::Ptr memento);

    virtual void setupUi(QBoxLayout* layout);

    void updateLabel(int label);

public Q_SLOTS:
    void display(const lib_laser_processing::Scan* img);
    void submit();
    void labelSelected();

Q_SIGNALS:
    void displayRequest(const lib_laser_processing::Scan* img);
    void submitRequest();

protected:
    bool eventFilter(QObject* o, QEvent* e);
    void labelSelected(int label);
    void updatePolygon();
    void labelInside();

    std::weak_ptr<ScanLabeler> wrapped_;

    struct State : public Memento {
        int width;
        int height;

        QPolygonF inside;
        QGraphicsPolygonItem* inside_item;

        State()
            : width(300), height(300)
        {}

        virtual void writeYaml(YAML::Node& out) const {
            out["width"] = width;
            out["height"] = height;

            std::vector<double> pts;
            for(const auto& pt : inside) {
                pts.push_back(pt.x());
                pts.push_back(pt.y());
            }
            out["inside"] = pts;
        }
        virtual void readYaml(const YAML::Node& node) {
            if(node["width"].IsDefined()) {
                width = node["width"].as<int>();
            }
            if(node["height"].IsDefined()) {
                height = node["height"].as<int>();
            }
            if(node["inside"].IsDefined()) {
                auto pts = node["inside"].as<std::vector<double>>();
                for(std::size_t i = 0; i < pts.size(); i+= 2) {
                    QPointF pt;
                    pt.setX(pts[i]);
                    pt.setY(pts[i+1]);
                    inside.push_back(pt);
                }
            }
        }
    };

private:
    constexpr static const double SCALE = 100.0;

private:
    connection_types::LabeledScanMessage::Ptr result_;

    QSize last_size_;
    State state;

    QGraphicsView* view_;

    bool resize_down_;
    bool move_down_;
    QPoint last_pos_;
};

}

#endif // SCAN_LABELER_ADAPTER_H
