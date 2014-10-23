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
    CloudRendererAdapter(NodeWorker *worker, CloudRenderer *node, WidgetController *widget_ctrl);
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
    void paintAugmentation();
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
    bool repaint_;

    QColor color_grid_;
    QColor color_bg_;
    QVector3D color_grad_start_;
    QVector3D color_grad_end_;

    bool size_sync_;
    int w_view_;
    int h_view_;
    int w_out_;
    int h_out_;

    double point_size_;
    double phi_;
    double theta_;
    double r_;

    bool axes_;

    int grid_size_;
    double grid_resolution_;
    bool grid_xy_;
    bool grid_yz_;
    bool grid_xz_;

    QVector3D offset_;

    GLuint list_cloud_;
    GLuint list_augmentation_;
};

}

#endif // CLOUD_RENDERER_ADAPTER_H
