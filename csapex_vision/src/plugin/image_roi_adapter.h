#ifndef ROI_IMAGE_ADAPTER_H
#define ROI_IMAGE_ADAPTER_H

/// PROJECT
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include "image_roi.h"

/// SYSTEM
#include <QGraphicsView>

namespace csapex {

class ImageRoiAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    ImageRoiAdapter(ImageRoi *node, WidgetController *widget_ctrl);

    virtual Memento::Ptr getState() const;
    virtual void         setParameterState(Memento::Ptr memento);

    virtual void         setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(QSharedPointer<QImage> img);
    void fitInView();
    void submit();
    void drop();

Q_SIGNALS:
    void displayRequest(QSharedPointer<QImage> img);
    void submitRequest();
    void dropRequest();

protected:
    bool eventFilter(QObject* o, QEvent* e);

    struct State : public Memento {
        int    width;
        int    height;

        State()
            : width(300), height(300)
        {}

        virtual void writeYaml(YAML::Node& out) const {
            out["width"] = width;
            out["height"] = height;
        }
        virtual void readYaml(const YAML::Node& node) {
            width = node["width"].as<int>();
            height = node["height"].as<int>();
        }
    };

    ImageRoi *wrapped_;

private:

    QSize                last_size_;
    QSize                last_roi_size_;
    State                state;

    QGraphicsPixmapItem *pixmap_;
    QGraphicsRectItem   *rect_;
    QRectF               roi_rect_;

    QGraphicsView*       view_;

    QImage               empty;
    QPainter             painter;

    bool                 middle_button_down_;
    bool                 left_button_down_;
    QPoint               middle_last_pos_;
    QPoint               left_last_pos_;
};

}

#endif // ROI_IMAGE_ADAPTER_H
