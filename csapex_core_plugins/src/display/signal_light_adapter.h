#ifndef SIGNAL_LIGHT_ADAPTER_H
#define SIGNAL_LIGHT_ADAPTER_H

/// PROJECT
#include <csapex/view/node/resizable_node_adapter.h>
#include <csapex/utility/yaml.h>

/// COMPONENT
#include "signal_light.h"

/// SYSTEM
#include <QLabel>
#include <QPainter>

namespace csapex
{
class LightWidget : public QWidget
{
    Q_OBJECT

public:
    LightWidget(const QColor& color, QWidget* parent = 0) : QWidget(parent), color_(color), enabled_(false)
    {
    }

    void setEnabled(bool on)
    {
        if (on != enabled_) {
            enabled_ = on;
            update();
        }
    }

protected:
    virtual void paintEvent(QPaintEvent*) override
    {
        if (!enabled_) {
            return;
        }

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setBrush(color_);
        int w = width();
        int h = height();
        int dim = std::min(w, h);

        int cx = (w - dim) / 2;
        int cy = (h - dim) / 2;

        QRadialGradient rg(cx, cy, dim);
        rg.setColorAt(0, color_);
        rg.setColorAt(1, QColor(color_).dark());
        painter.setBrush(QBrush(rg));

        painter.drawEllipse(cx, cy, dim, dim);
    }

private:
    QColor color_;
    bool enabled_;
};

class SignalLightWidget : public QWidget
{
    Q_OBJECT

public:
    SignalLightWidget(QWidget* parent = 0) : QWidget(parent)
    {
        QVBoxLayout* vbox = new QVBoxLayout(this);

        red_ = new LightWidget(Qt::red);
        vbox->addWidget(red_);

        yellow_ = new LightWidget(Qt::yellow);
        vbox->addWidget(yellow_);

        green = new LightWidget(Qt::green);
        vbox->addWidget(green);

        QPalette pal = palette();
        pal.setColor(QPalette::Background, Qt::black);
        setPalette(pal);
        setAutoFillBackground(true);
    }

    LightWidget* red() const
    {
        return red_;
    }
    LightWidget* yellow() const
    {
        return yellow_;
    }
    LightWidget* gren() const
    {
        return green;
    }

private:
    LightWidget* red_;
    LightWidget* yellow_;
    LightWidget* green;
};

class SignalLightAdapter : public QObject, public ResizableNodeAdapter
{
    Q_OBJECT

public:
    SignalLightAdapter(NodeFacadePtr node, NodeBox* parent);
    ~SignalLightAdapter();

    void setupUi(QBoxLayout* layout) override;

    void setManualResize(bool manual) override;

    void resize(const QSize& size) override;

public Q_SLOTS:
    void display(int state);

Q_SIGNALS:
    void displayRequest(int state);

private:
    SignalLightWidget* light_;
};

}  // namespace csapex
#endif  // SIGNAL_LIGHT_ADAPTER_H
