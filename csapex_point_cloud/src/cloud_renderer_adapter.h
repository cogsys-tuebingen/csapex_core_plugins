#ifndef CLOUD_RENDERER_ADAPTER_H
#define CLOUD_RENDERER_ADAPTER_H

/// PROJECT
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include "cloud_renderer.h"

/// SYSTEM
#include <QGLWidget>
#include <QVector3D>

namespace csapex {

class CloudRendererAdapter : public QGLWidget, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    CloudRendererAdapter(CloudRenderer *node, WidgetController *widget_ctrl);

    virtual void setupUi(QBoxLayout* layout);

    void display();
    void refresh();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent* event);

public:
    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

public Q_SLOTS:
    void setTheta(double angle);
    void setPhi(double angle);

    void displayCloud();

Q_SIGNALS:
    void thetaChanged(double angle);
    void phiChanged(double angle);

    void displayRequest();
    void repaintRequest();

protected:
    CloudRenderer* wrapped_;

    bool drag_;
    QPoint last_pos_;

    QColor color_bg_;
    QVector3D color_grad_start_;
    QVector3D color_grad_end_;

    int w_;
    int h_;

    double point_size_;
    double phi_;
    double theta_;
    double r_;

    QVector3D offset_;

    GLuint list_;
};

}

#endif // CLOUD_RENDERER_ADAPTER_H
