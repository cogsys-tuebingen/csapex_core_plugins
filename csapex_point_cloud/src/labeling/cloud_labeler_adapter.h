#ifndef CLOUD_LABELER_ADAPTER_H
#define CLOUD_LABELER_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "cloud_labeler.h"

/// SYSTEM
#include <QGLWidget>
#include <QVector3D>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGLFramebufferObject>
#include <QMatrix4x4>

namespace csapex {

class CloudLabelerAdapter : public QGLWidget, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    CloudLabelerAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<CloudLabeler> node);
    ~CloudLabelerAdapter();

    void stop();

    virtual void setupUi(QBoxLayout* layout);

    void display();
    void refresh();
    void done();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    void updateLabel(int labelArea);

protected:
    void initializeGL();
    void resizeGL(int width, int height);
    void mousePressEventImpl(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEventImpl(QGraphicsSceneMouseEvent *event);
    void mouseMoveEventImpl(QGraphicsSceneMouseEvent *event);
    void wheelEventImpl(QGraphicsSceneWheelEvent *event);

    bool eventFilter(QObject *, QEvent *);

    std::pair<QVector3D, QVector3D> calculateRay(const QPointF &cursor);

    void drawPoints();

    void drawSphere(double r, int lats, int longs);

public:
    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

public Q_SLOTS:
    void setTheta(double angle);
    void setPhi(double angle);

    void displayCloud();
    void paintGLImpl(bool request = true);
    void paintAugmentation();
    void resize();

    void labelArea();
    void labelPoint();

Q_SIGNALS:
    void thetaChanged(double angle);
    void phiChanged(double angle);

    void displayRequest();
    void repaintRequest();
    void resizeRequest();

protected:
    std::weak_ptr<CloudLabeler> wrapped_;
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_;

    QGraphicsView* view_;
    QGraphicsPixmapItem* pixmap_;
    QGraphicsRectItem* selection_;
    QGLFramebufferObject* fbo_;

    bool drag_;
    QPointF last_pos_;
    bool repaint_;

    QColor color_grid_;
    QColor color_bg_;

    QVector3D eye;
    QVector3D up;
    QVector3D look_at_pt;

    QMatrix4x4 projection;
    QMatrix4x4 modelview;

    double fov_v_;
    double near_;
    double far_;

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

    bool label_rect_;
    bool label_point_;
    QPointF cursor_;
    QPointF cursor_label_start_;
    QPointF cursor_label_end_;

    QVector3D selection_3d_cursor_;
    double radius_;

    std::pair<QVector3D, QVector3D> selection_a_;
    std::pair<QVector3D, QVector3D> selection_b_;
    std::pair<QVector3D, QVector3D> selection_c_;
    std::pair<QVector3D, QVector3D> selection_d_;

    QVector3D selection_eye_;
};

}

#endif // CLOUD_LABELER_ADAPTER_H
