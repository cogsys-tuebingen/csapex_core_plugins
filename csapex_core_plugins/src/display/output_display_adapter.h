#ifndef OUTPUT_DISPLAY_ADAPTER_H
#define OUTPUT_DISPLAY_ADAPTER_H

/// PROJECT
#include <csapex/view/node/resizable_node_adapter.h>

/// COMPONENT
#include "output_display.h"

/// SYSTEM
#include <QGraphicsView>
#include <QLabel>
#include <yaml-cpp/yaml.h>

namespace csapex {

class ImageWidget;

class OutputDisplayAdapter : public QObject, public ResizableNodeAdapter
{
    Q_OBJECT

public:
    OutputDisplayAdapter(NodeFacadeWeakPtr worker, NodeBox* parent, std::weak_ptr<OutputDisplay> node);
    ~OutputDisplayAdapter();


    virtual void setupUi(QBoxLayout* layout) override;

    virtual void setManualResize(bool manual) override;

public Q_SLOTS:
    void display(const QImage& img);
    void fitInView();

Q_SIGNALS:
    void displayRequest(QImage img);

protected:
    bool eventFilter(QObject* o, QEvent* e);
    virtual void resize(const QSize& size) override;

    std::weak_ptr<OutputDisplay> wrapped_;

private:

    QSize last_image_size_;

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

#endif // OUTPUT_DISPLAY_ADAPTER_H
