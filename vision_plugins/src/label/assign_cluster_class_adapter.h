#ifndef ASSIGN_CLUSTER_CLASS_ADAPTER_H
#define ASSIGN_CLUSTER_CLASS_ADAPTER_H

/// PROJECT
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include "assign_cluster_class.h"

/// SYSTEM
#include <QGraphicsView>
#include <QImage>

namespace csapex {

class AssignClusterClassAdapter : public QObject, public csapex::DefaultNodeAdapter
{
    Q_OBJECT

public:
    AssignClusterClassAdapter(csapex::NodeWorker *worker,
                              vision_plugins::AssignClusterClass *node,
                              csapex::WidgetController *widget_ctrl);

    virtual csapex::Memento::Ptr getState() const;
    virtual void                 setParameterState(csapex::Memento::Ptr memento);

    virtual void                 setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(QSharedPointer<QImage> img, const cv::Mat &clusters);
    void fitInView();
    void submit();
    void drop();
    void setColor(int r,int g, int b);
    void setClass(int c);

Q_SIGNALS:
    void displayRequest(QSharedPointer<QImage> img, const cv::Mat &clusters);
    void submitRequest();
    void dropRequest();
    void setColorRequest(int r,int g, int b);
    void setClassRequest(int c);

protected:
    bool eventFilter(QObject* o, QEvent* e);
    void updateClusterClass(const QPoint &pos);


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


    vision_plugins::AssignClusterClass *wrapped_;

private:
    std::vector<int>      classes_;
    std::map<int, QColor> colors_;
    int                   active_class_;


    State                  state;

    QSharedPointer<QImage> img_;
    QSharedPointer<QImage> overlay_;

    QGraphicsPixmapItem   *pixmap_overlay_;
    QGraphicsPixmapItem   *pixmap_;
    cv::Mat                clusters_;


    QGraphicsView*       view_;

    QImage               empty;
    QPainter             painter;

    bool                 middle_button_down_;
    bool                 loaded_;
    QPoint               middle_last_pos_;
    QPoint               left_last_pos_;
};

}

#endif // ASSIGN_CLUSTER_CLASS_ADAPTER_H
