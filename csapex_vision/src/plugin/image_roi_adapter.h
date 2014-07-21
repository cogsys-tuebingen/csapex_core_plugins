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
    virtual void         setState(Memento::Ptr memento);

    virtual void         setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(QSharedPointer<QImage> img);
    void fitInView();
    void submit();


Q_SIGNALS:
    void displayRequest(QSharedPointer<QImage> img);
    void submitRequest();

protected:
    bool eventFilter(QObject* o, QEvent* e);

    struct State : public Memento {
        int    width;
        int    height;

        State()
            : width(300), height(300)
        {}

        virtual void writeYaml(YAML::Emitter& out) const {
            out << YAML::Key << "width" << YAML::Value << width;
            out << YAML::Key << "height" << YAML::Value << height;
        }
        virtual void readYaml(const YAML::Node& node) {
            node["width"]  >> width;
            node["height"] >> height;
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
