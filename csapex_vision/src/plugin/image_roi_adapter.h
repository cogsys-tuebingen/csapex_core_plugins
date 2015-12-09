#ifndef ROI_IMAGE_ADAPTER_H
#define ROI_IMAGE_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "image_roi.h"

/// SYSTEM
#include <QGraphicsView>

namespace csapex {

class ImageRoiAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    ImageRoiAdapter(NodeHandleWeakPtr worker, std::weak_ptr<ImageRoi> node, WidgetController *widget_ctrl);

    virtual Memento::Ptr getState() const;
    virtual void         setParameterState(Memento::Ptr memento);

    virtual void         setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(const QImage &img);
    void fitInView();
    void submit();
    void drop();

Q_SIGNALS:
    void displayRequest(QImage img);
    void submitRequest();
    void dropRequest();

protected:
    bool eventFilter(QObject* o, QEvent* e);

    struct State : public Memento {
        int     width;
        int     height;
        QSize   last_size;
        QSize   last_roi_size;
        QRectF  roi_rect;
        QPointF scene_pos;

        State()
            : width(300), height(300),
              last_size(-1,-1),
              last_roi_size(-1,-1)
        {}

        virtual void writeYaml(YAML::Node& out) const {
            out["width"]           = width;
            out["height"]          = height;
            out["last_width"]      = last_size.width();
            out["last_height"]     = last_size.height();
            out["last_roi_height"] = last_roi_size.height();
            out["last_roi_width"]  = last_roi_size.width();
            out["roi_r_w"]         = roi_rect.width();
            out["roi_r_h"]         = roi_rect.height();
            out["scene_r_x"]       = scene_pos.x();
            out["scene_r_y"]       = scene_pos.y();
        }

        virtual void readYaml(const YAML::Node& node) {
            width = node["width"].as<int>();
            height = node["height"].as<int>();
            last_size.setHeight(node["last_height"].as<float>());
            last_size.setWidth(node["last_width"].as<float>());
            last_roi_size.setHeight(node["last_roi_height"].as<float>());
            last_roi_size.setWidth(node["last_roi_width"].as<float>());
            roi_rect.setWidth(node["roi_r_w"].as<float>());
            roi_rect.setHeight(node["roi_r_h"].as<float>());
            scene_pos.setX(node["scene_r_x"].as<float>());
            scene_pos.setY(node["scene_r_y"].as<float>());
        }

    };

    std::weak_ptr<ImageRoi> wrapped_;

private:
    State                state;

    QGraphicsPixmapItem *pixmap_;
    QGraphicsRectItem   *rect_;

    QGraphicsView*       view_;

    QImage               empty;
    QPainter             painter;

    bool                 middle_button_down_;
//    bool                 left_button_down_;
    bool                 loaded_;
    QPoint               middle_last_pos_;
    QPoint               left_last_pos_;
};

}

#endif // ROI_IMAGE_ADAPTER_H
