#ifndef OUTPUD_DISPLAY_ADAPTER_H
#define OUTPUD_DISPLAY_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "output_display.h"

/// SYSTEM
#include <QGraphicsView>
#include <QLabel>
#include <yaml-cpp/yaml.h>

namespace csapex {

class ImageWidget;

class OutputDisplayAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    OutputDisplayAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<OutputDisplay> node);
    ~OutputDisplayAdapter();

    virtual MementoPtr getState() const override;
    virtual void setParameterState(Memento::Ptr memento) override;

    virtual void setupUi(QBoxLayout* layout) override;

    virtual void setManualResize(bool manual) override;
    virtual bool isResizable() const override;

public Q_SLOTS:
    void display(const QImage& img);
    void fitInView();

Q_SIGNALS:
    void displayRequest(QImage img);

protected:
    bool eventFilter(QObject* o, QEvent* e);
    void resize();

    std::weak_ptr<OutputDisplay> wrapped_;

    struct State : public Memento {
        int width;
        int height;

        State()
            : width(100), height(100)
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

    ImageWidget* label_view_;
};


class ImageWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImageWidget(QWidget *parent = 0);
    const QPixmap* pixmap() const;

    void setManualResize(bool manual);

    void setSize(const QSize& size);
    void setSize(int w, int h);

    virtual QSize sizeHint() const override;
    virtual QSize minimumSizeHint() const override;

public Q_SLOTS:
    void setPixmap(const QPixmap&);

protected:
    void paintEvent(QPaintEvent *) override;
    void resizeEvent(QResizeEvent*) override;

private:
    QPixmap pix;
    QSize size;

    bool manual_resize_;
};

}

#endif // OUTPUD_DISPLAY_ADAPTER_H
