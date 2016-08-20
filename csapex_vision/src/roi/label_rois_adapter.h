#ifndef ASSIGN_ROI_CLASS_ADAPTER_H
#define ASSIGN_ROI_CLASS_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "label_rois.h"

/// SYSTEM
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QImage>
#include <yaml-cpp/yaml.h>

namespace csapex {

class QInteractiveRect;

class LabelROIsAdapter : public QObject, public csapex::DefaultNodeAdapter
{
    Q_OBJECT

public:
    LabelROIsAdapter(csapex::NodeHandleWeakPtr worker, csapex::NodeBox* parent,
                              std::weak_ptr<LabelROIs> node);

    virtual csapex::Memento::Ptr getState() const;
    virtual void                 setParameterState(csapex::Memento::Ptr memento);

    virtual void                 setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(QImage img);
    void fitInView();
    void submit();
    void drop();
    void clear();
    void setColor(int r,int g, int b);
    void setClass(int c);

Q_SIGNALS:
    void displayRequest(QImage img);
    void submitRequest();
    void dropRequest();
    void clearRequest();
    void setColorRequest(int r,int g, int b);
    void setClassRequest(int c);

protected:
    bool eventFilter(QObject* o, QEvent* e);

    struct State : public csapex::Memento {
        int     width;
        int     height;
        QSize   last_size;
        QSize   last_roi_size;
        QRectF  roi_rect;
        QPointF scene_pos;

        State()
            : width(300), height(300),
              last_size(-1,-1)
        {}

        virtual void writeYaml(YAML::Node& out) const {
            out["width"]           = width;
            out["height"]          = height;
            out["last_width"]      = last_size.width();
            out["last_height"]     = last_size.height();
            out["scene_r_x"]       = scene_pos.x();
            out["scene_r_y"]       = scene_pos.y();
        }

        virtual void readYaml(const YAML::Node& node) {
            width = node["width"].as<int>();
            height = node["height"].as<int>();
            last_size.setHeight(node["last_height"].as<float>());
            last_size.setWidth(node["last_width"].as<float>());
            scene_pos.setX(node["scene_r_x"].as<float>());
            scene_pos.setY(node["scene_r_y"].as<float>());
        }

    };


    std::weak_ptr<csapex::LabelROIs> wrapped_;

private:
    std::map<int, QColor> colors_;
    int                   active_class_;
    QColor                active_color_;

    State                  state;

    QImage img_;
    QGraphicsPixmapItem   *pixmap_;

    std::vector<QInteractiveRect*> rectangles_;

    QGraphicsView*       view_;

    QImage               empty;
    QPainter             painter;

    bool                 middle_button_down_;
    bool                 left_button_down_;
    bool                 loaded_;
    QPoint               middle_last_pos_;
};

}

#endif // ASSIGN_ROI_CLASS_ADAPTER_H
