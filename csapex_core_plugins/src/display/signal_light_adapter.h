#ifndef SIGNAL_LIGHT_ADAPTER_H
#define SIGNAL_LIGHT_ADAPTER_H


/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "signal_light.h"

/// SYSTEM
#include <QLabel>
#include <QPainter>
#include <yaml-cpp/yaml.h>

namespace csapex {

class LightWidget : public QWidget
{
    Q_OBJECT

public:
    LightWidget(const QColor &color, QWidget *parent = 0)
        : QWidget(parent), color_(color), enabled_(false) {}

    void setEnabled(bool on)
    {
        if (on != enabled_) {
            enabled_ = on;
            update();
        }
    }

protected:
    virtual void paintEvent(QPaintEvent *) override
    {
        if (!enabled_) {
            return;
        }

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setBrush(color_);
        int w = width();
        int h = height();
        int dim = std::min(w,h);

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
    SignalLightWidget(QWidget *parent = 0)
        : QWidget(parent)
    {
        QVBoxLayout *vbox = new QVBoxLayout(this);

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

    LightWidget *red() const
    {
        return red_;
    }
    LightWidget *yellow() const
    {
        return yellow_;
    }
    LightWidget *gren() const
    {
        return green;
    }

private:
    LightWidget *red_;
    LightWidget *yellow_;
    LightWidget *green;
};

class SignalLightAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    SignalLightAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<SignalLight> node);

    virtual Memento::Ptr getState() const override;
    virtual void setParameterState(Memento::Ptr memento) override;

    virtual void setupUi(QBoxLayout* layout);

    virtual bool isResizable() const override;
    virtual void setManualResize(bool manual) override;

public Q_SLOTS:
    void display(int state);

Q_SIGNALS:
    void displayRequest(int state);

protected:
    std::weak_ptr<SignalLight> wrapped_;

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
    SignalLightWidget* light_;
    State state;
};

}
#endif // SIGNAL_LIGHT_ADAPTER_H
