#ifndef OUTPUD_DISPLAY_ADAPTER_H
#define OUTPUD_DISPLAY_ADAPTER_H

/// PROJECT
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include "output_display.h"

/// SYSTEM
#include <QGraphicsView>
#include <yaml-cpp/yaml.h>

namespace csapex {

class OutputDisplayAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    OutputDisplayAdapter(NodeWorker *worker, OutputDisplay *node, WidgetController *widget_ctrl);

    virtual Memento::Ptr getState() const;
    virtual void setParameterState(Memento::Ptr memento);

    virtual void setupUi(QBoxLayout* layout);

public Q_SLOTS:
    void display(QSharedPointer<QImage> img);
    void fitInView();

Q_SIGNALS:
    void displayRequest(QSharedPointer<QImage> img);

protected:
    bool eventFilter(QObject* o, QEvent* e);

    OutputDisplay* wrapped_;

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
            width = node["width"].as<int>();
            height = node["height"].as<int>();
        }
    };


private:
    QSize last_size_;
    State state;

    QGraphicsPixmapItem* pixmap_;

    QGraphicsView* view_;

    QImage empty;
    QPainter painter;

    bool down_;
    QPoint last_pos_;
};

}

#endif // OUTPUD_DISPLAY_ADAPTER_H
