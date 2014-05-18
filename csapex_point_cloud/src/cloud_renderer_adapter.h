#ifndef CLOUD_RENDERER_ADAPTER_H
#define CLOUD_RENDERER_ADAPTER_H

/// PROJECT
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include "cloud_renderer.h"

/// SYSTEM
#include <QGLWidget>
#include <QVector3D>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGLFramebufferObject>

namespace csapex {

class CloudRendererAdapter : public QGLWidget, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    CloudRendererAdapter(CloudRenderer *node, WidgetController *widget_ctrl);
    ~CloudRendererAdapter();

    void stop();

    virtual void setupUi(QBoxLayout* layout);

    void display();
    void refresh();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

protected:
    void initializeGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void wheelEvent(QGraphicsSceneWheelEvent *event);

    bool eventFilter(QObject *, QEvent *);

public:
    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

public Q_SLOTS:
    void setTheta(double angle);
    void setPhi(double angle);

    void displayCloud();
    void paintGL(bool request = true);
    void resize();

Q_SIGNALS:
    void thetaChanged(double angle);
    void phiChanged(double angle);

    void displayRequest();
    void repaintRequest();
    void resizeRequest();

protected:
    CloudRenderer* wrapped_;

    QGraphicsView* view_;
    QGraphicsPixmapItem* pixmap_;
    QGLFramebufferObject* fbo_;

    bool drag_;
    QPointF last_pos_;

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
