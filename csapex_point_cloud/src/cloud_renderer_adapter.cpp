/// HEADER
#include "cloud_renderer_adapter.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_node_adapter.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/q_signal_relay.h>

/// SYSTEM
#include <QtOpenGL>
#include <utils_qt/QtCvImageConverter.h>
#include <pcl/for_each_type.h>
#include <pcl/conversions.h>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_NODE_ADAPTER(CloudRendererAdapter, csapex::CloudRenderer)

CloudRendererAdapter::CloudRendererAdapter(CloudRenderer *node, WidgetController* widget_ctrl)
    : QGLWidget(QGLFormat(QGL::SampleBuffers)), DefaultNodeAdapter(node, widget_ctrl),
      wrapped_(node), view_(NULL), pixmap_(NULL), fbo_(NULL), drag_(false), repaint_(true),
      w_view_(10), h_view_(10), point_size_(1),
      phi_(0), theta_(M_PI/2), r_(-10.0),
      axes_(false), grid_size_(10), grid_resolution_(1.0), grid_xy_(true), grid_yz_(false), grid_xz_(false),
      list_cloud_(0), list_augmentation_(0)
{
    node->display_request.connect(boost::bind(&CloudRendererAdapter::display, this));
    node->refresh_request.connect(boost::bind(&CloudRendererAdapter::refresh, this));

    QObject::connect(this, SIGNAL(repaintRequest()), this, SLOT(paintGL()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(resizeRequest()), this, SLOT(resize()), Qt::QueuedConnection);
}

CloudRendererAdapter::~CloudRendererAdapter()
{
    delete fbo_;
}

void CloudRendererAdapter::stop()
{
    DefaultNodeAdapter::stop();
    disconnect();
}

void CloudRendererAdapter::setupUi(QBoxLayout* layout)
{
    view_ = new QGraphicsView;
    QGraphicsScene* scene = view_->scene();
    if(scene == NULL) {
        scene = new QGraphicsScene();
        //scene->addWidget(this);
        view_->setScene(scene);
        view_->setMouseTracking(true);
    }

    view_->setContextMenuPolicy(Qt::PreventContextMenu);

    scene->installEventFilter(this);

    layout->addWidget(view_);

    QObject::connect(this, SIGNAL(displayRequest()), this, SLOT(displayCloud()));

    DefaultNodeAdapter::setupUi(layout);
}



void CloudRendererAdapter::initializeGL()
{
    makeCurrent();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glDisable(GL_LIGHTING);
    glEnable(GL_MULTISAMPLE);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void CloudRendererAdapter::resizeGL(int width, int height)
{
    makeCurrent();

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    QMatrix4x4 projection;
    projection.setToIdentity();
    projection.perspective(45.0f,(GLfloat)width/(GLfloat)height,0.0001f,300.0f);
    glLoadMatrixd(projection.data());
    glMatrixMode(GL_MODELVIEW);
}

void CloudRendererAdapter::resize()
{
    if(w_view_ != view_->width() || h_view_ != view_->height()) {
        view_->setFixedSize(w_view_, h_view_);
    }
    resizeGL(w_out_, h_out_);
}

void CloudRendererAdapter::paintAugmentation()
{
    makeCurrent();

    if(list_augmentation_ == 0) {
        list_augmentation_ = glGenLists(1);
    }

    // push settings
    glPushAttrib(GL_CULL_FACE);
    glPushAttrib(GL_LINE_SMOOTH);
    glPushAttrib(GL_BLEND);

    glNewList(list_augmentation_,GL_COMPILE);

    // change settings
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glDisable( GL_CULL_FACE );
    glEnable(GL_LINE_SMOOTH);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);


    // grid
    qglColor(color_grid_);

    glLineWidth(1.5f);
    glBegin(GL_QUADS);
    double dim = grid_resolution_ * grid_size_ / 2.0;
    double r = grid_resolution_;
    if(grid_xy_) {
        for(double x = -dim; x < dim; x += grid_resolution_) {
            for(double y = -dim; y < dim; y += grid_resolution_) {
                glVertex3d(x,y,0);
                glVertex3d(x+r,y,0);
                glVertex3d(x+r,y+r,0);
                glVertex3d(x,y+r,0);
            }
        }
    }
    if(grid_yz_) {
        for(double y = -dim; y < dim; y += grid_resolution_) {
            for(double z = -dim; z < dim; z += grid_resolution_) {
                glVertex3d(0,y,z);
                glVertex3d(0,y+r,z);
                glVertex3d(0,y+r,z+r);
                glVertex3d(0,y,z+r);
            }
        }
    }
    if(grid_xz_) {
        for(double x = -dim; x < dim; x += grid_resolution_) {
            for(double z = -dim; z < dim; z += grid_resolution_) {
                glVertex3d(x,0,z);
                glVertex3d(x+r,0,z);
                glVertex3d(x+r,0,z+r);
                glVertex3d(x,0,z+r);
            }
        }
    }
    glEnd();

    if(axes_) {
        // axes
        double d = 0.5;
        glLineWidth(20.f);
        glBegin(GL_LINES);
        // x
        glColor3d(1,0,0);
        glVertex3d(0,0,0);
        glVertex3d(d,0,0);
        // y
        glColor3d(0,1,0);
        glVertex3d(0,0,0);
        glVertex3d(0,d,0);
        // z
        glColor3d(0,0,1);
        glVertex3d(0,0,0);
        glVertex3d(0,0,d);
        glEnd();
    }


    glEndList();

    // pop settings
    glPopAttrib();
    glPopAttrib();
    glPopAttrib();
}

void CloudRendererAdapter::paintGL(bool request)
{
    // initialization
    makeCurrent();
    initializeGL();

    if(fbo_) {
        delete fbo_;
    }

    QGLFramebufferObjectFormat format;
    format.setAttachment(QGLFramebufferObject::CombinedDepthStencil);
    format.setMipmap(false);
    format.setInternalTextureFormat(GL_RGB8);
    fbo_ = new QGLFramebufferObject(QSize(w_out_, h_out_), format);
    fbo_->bind();

    qglClearColor(color_bg_);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // model view matrix
    QMatrix4x4 lookat;
    lookat.setToIdentity();
    QVector3D eye(-r_ * std::sin(theta_) * std::cos(phi_),
                  -r_ * std::sin(theta_) * std::sin(phi_),
                  -r_ * std::cos(theta_));
    QVector3D center(0,0,0);
    QVector3D up(0,0,1);
    lookat.lookAt(eye + offset_, center + offset_, up);
    glLoadMatrixd(lookat.data());

    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glCallList(list_cloud_);

    // grid
    if(repaint_ || !list_augmentation_) {
        paintAugmentation();
    }
    glCallList(list_augmentation_);


    // center point
    if(drag_) {
        QVector3D o = center + offset_;

        glPointSize(point_size_ * 5);
        glBegin(GL_POINTS);
        glColor3f(1,1,1);
        glVertex3d(o.x(), o.y(), o.z());
        glEnd();
    }

    // extract image
    QImage img = fbo_->toImage();

    fbo_->release();

    if(pixmap_ == NULL) {
        pixmap_ = view_->scene()->addPixmap(QPixmap::fromImage(img));
    } else {
        pixmap_->setPixmap(QPixmap::fromImage(img));
    }

    view_->blockSignals(true);
    view_->scene()->setSceneRect(img.rect());
    view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
    view_->blockSignals(false);
    //    view_->scene()->update();

    if(wrapped_->output_->isConnected() && request){
        cv::Mat mat = QtCvImageConverter::Converter<QImage, QSharedPointer>::QImage2Mat(img);
        wrapped_->publishImage(mat);
    }
}

bool CloudRendererAdapter::eventFilter(QObject * o, QEvent * e)
{
    if(view_->signalsBlocked()) {
        return false;
    }

    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::GraphicsSceneMousePress:
        mousePressEvent(me);
        return true;
    case QEvent::GraphicsSceneMouseRelease:
        mouseReleaseEvent(me);
        return true;
    case QEvent::GraphicsSceneMouseMove:
        mouseMoveEvent(me);
        return true;
    case QEvent::GraphicsSceneWheel:
        wheelEvent(dynamic_cast<QGraphicsSceneWheelEvent*>(e));
        return true;

    default:
        break;
    }

    return false;
}

void CloudRendererAdapter::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    last_pos_ = event->screenPos();
    drag_ = true;
    event->accept();
}

void CloudRendererAdapter::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    drag_ = false;

    wrapped_->getParameter("~size/width")->set<int>(w_view_);
    wrapped_->getParameter("~size/height")->set<int>(h_view_);

    if(size_sync_) {
        w_out_ = w_view_;
        h_out_ = h_view_;
        wrapped_->getParameter("~size/out/width")->set<int>(w_out_);
        wrapped_->getParameter("~size/out/height")->set<int>(h_out_);
    }
    event->accept();
}

void CloudRendererAdapter::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if(!drag_) {
        return;
    }

    event->accept();

    QPointF pos = event->screenPos();
    double dx = pos.x() - last_pos_.x();
    double dy = pos.y() - last_pos_.y();

    double f = 0.01;

    if (event->buttons() & Qt::LeftButton) {
        setTheta(theta_ + f * dy);
        setPhi(phi_ + f * -dx);

    } else if (event->buttons() & Qt::MidButton) {
        w_view_ = std::max(40, std::min(2000, w_view_ + (int) dx));
        h_view_ = std::max(40, std::min(2000, h_view_ + (int) dy));

        view_->setFixedSize(w_view_,h_view_);
        if(size_sync_) {
            w_out_ = w_view_;
            h_out_ = h_view_;

            resizeGL(w_out_, h_out_);
        }

    } else if (event->buttons() & Qt::RightButton) {
        QMatrix4x4 rot;
        rot.setToIdentity();
        rot.rotate(phi_ * 180.0 / M_PI, QVector3D(0,0,1));
        offset_ += rot * QVector3D(f*dy, f*dx, 0);


        wrapped_->getParameter("~view/dx")->set<double>(offset_.x());
        wrapped_->getParameter("~view/dy")->set<double>(offset_.y());
        wrapped_->getParameter("~view/dz")->set<double>(offset_.z());

    }
    last_pos_ = pos;
    paintGL(false);
}

void CloudRendererAdapter::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    event->accept();

    r_ += event->delta() * -0.0025;
    wrapped_->getParameter("~view/r")->set<double>(r_);
    paintGL(false);

}


void CloudRendererAdapter::display()
{
    Q_EMIT displayRequest();
}

void CloudRendererAdapter::refresh()
{
    if(!drag_){
        {
            const std::vector<int>& c = wrapped_->param<std::vector<int> >("color/background");
            color_bg_ = QColor::fromRgb(c[0], c[1], c[2]);
        }
        {
            const std::vector<int>& c = wrapped_->param<std::vector<int> >("color/grid");
            color_grid_ = QColor::fromRgb(c[0], c[1], c[2]);
        }
        {
            const std::vector<int>& c = wrapped_->param<std::vector<int> >("color/gradient/start");
            color_grad_start_ = QVector3D(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0);
        }
        {
            const std::vector<int>& c = wrapped_->param<std::vector<int> >("color/gradient/end");
            color_grad_end_ = QVector3D(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0);
        }
        r_ = wrapped_->param<double>("~view/r");
        theta_ = wrapped_->param<double>("~view/theta");
        phi_ = wrapped_->param<double>("~view/phi");

        double dx = wrapped_->param<double>("~view/dx");
        double dy = wrapped_->param<double>("~view/dy");
        double dz = wrapped_->param<double>("~view/dz");

        offset_ = QVector3D(dx,dy,dz);
        point_size_ = wrapped_->param<double>("point/size");

        size_sync_ = wrapped_->param<bool>("~size/out/sync");
        w_view_ = wrapped_->param<int>("~size/width");
        h_view_ = wrapped_->param<int>("~size/height");

        if(size_sync_) {
            w_out_ = w_view_;
            h_out_ = h_view_;
        } else {
            w_out_ = wrapped_->param<int>("~size/out/width");
            h_out_ = wrapped_->param<int>("~size/out/height");
        }

        axes_ = wrapped_->param<bool>("show axes");

        grid_size_ = wrapped_->param<int>("~grid/size");
        grid_resolution_ = wrapped_->param<double>("~grid/resolution");
        grid_xy_ = wrapped_->param<bool>("~grid/xy");
        grid_yz_ = wrapped_->param<bool>("~grid/yz");
        grid_xz_ = wrapped_->param<bool>("~grid/xz");

        repaint_ = true;

        Q_EMIT resizeRequest();

        Q_EMIT repaintRequest();
    }
}

void CloudRendererAdapter::displayCloud()
{
    boost::apply_visitor (PointCloudMessage::Dispatch<CloudRendererAdapter>(this, wrapped_->message_), wrapped_->message_->value);
}

namespace
{

enum Component
{
    X, Y, Z
};


template <class PointT, int Component>
struct Access
{
};

template <class PointT>
struct Access<PointT, X>
{
    static double access(typename pcl::PointCloud<PointT>::iterator it)
    {
        return it->x;
    }
};

template <class PointT>
struct Access<PointT, Y>
{
    static double access(typename pcl::PointCloud<PointT>::iterator it)
    {
        return it->y;
    }
};

template <class PointT>
struct Access<PointT, Z>
{
    static double access(typename pcl::PointCloud<PointT>::iterator it)
    {
        return it->z;
    }
};

template <class PointT, int Component>
struct Util
{
    static void findExtrema(typename pcl::PointCloud<PointT>::Ptr cloud, double& min, double& max)
    {
        min = std::numeric_limits<double>::max();
        max = std::numeric_limits<double>::min();
        for(typename std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            if(Access<PointT, Component>::access(it) < min) {
                min = Access<PointT, Component>::access(it);
            }
            if(Access<PointT, Component>::access(it) > max) {
                max = Access<PointT, Component>::access(it);
            }
        }
    }
};
template <class PointT, int Component>
struct RendererGradient
{
    static void render(typename pcl::PointCloud<PointT>::Ptr cloud, const QVector3D& color_grad_start, const QVector3D& color_grad_end)
    {
        double min, max;
        Util<PointT, Component>::findExtrema(cloud, min, max);

        for(typename std::vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            PointT& pt = *it;

            double v = Access<PointT, Component>::access(it);
            double f = std::min(1.0, std::max(0.0, (v - min) / (max -min)));
            QVector3D c = f * color_grad_start + (1.0-f) * color_grad_end;

            glColor3d(c.x(), c.y(), c.z());

            glVertex3d(pt.x, pt.y, pt.z);
        }
    }
};

template <class PointT, int Component>
struct Renderer
{
    static void render(typename pcl::PointCloud<PointT>::Ptr cloud, const QVector3D& color_grad_start, const QVector3D& color_grad_end)
    {
        RendererGradient<PointT, Component>::render(cloud, color_grad_start, color_grad_end);
    }
};

template <int Component>
struct Renderer<pcl::PointXYZRGB, Component>
{
    static void render(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const QVector3D&, const QVector3D&)
    {
        for(typename std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            pcl::PointXYZRGB& pt = *it;

            glColor3d(pt.r, pt.g, pt.b);

            glVertex3d(pt.x, pt.y, pt.z);
        }
    }
};
}

template <class PointT>
void CloudRendererAdapter::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    makeCurrent();

    if(list_cloud_ == 0) {
        list_cloud_ = glGenLists(1);
    }

    glNewList(list_cloud_,GL_COMPILE);

    glPointSize(point_size_);

    glBegin(GL_POINTS);

    std::string component = wrapped_->param<std::string>("color/field");

    if(wrapped_->param<bool>("color/force gradient")) {
        if(component == "x") {
            RendererGradient<PointT, X>::render(cloud, color_grad_start_, color_grad_end_);
        } else if(component == "y") {
            RendererGradient<PointT, Y>::render(cloud, color_grad_start_, color_grad_end_);
        } else if(component == "z") {
            RendererGradient<PointT, Z>::render(cloud, color_grad_start_, color_grad_end_);
        }

    } else {
        if(component == "x") {
            Renderer<PointT, X>::render(cloud, color_grad_start_, color_grad_end_);
        } else if(component == "y") {
            Renderer<PointT, Y>::render(cloud, color_grad_start_, color_grad_end_);
        } else if(component == "z") {
            Renderer<PointT, Z>::render(cloud, color_grad_start_, color_grad_end_);
        }
    }

    glEnd();

    glEndList();

    paintGL();
}


QSize CloudRendererAdapter::minimumSizeHint() const
{
    return QSize(10, 10);
}

QSize CloudRendererAdapter::sizeHint() const
{
    return QSize(w_view_, h_view_);
}

namespace {
double normalizeAngle(double angle) {
    while(angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    while(angle > M_PI) {
        angle -= 2 * M_PI;
    }

    return angle;
}
}

void CloudRendererAdapter::setTheta(double angle)
{
    double eps = 1e-3;
    if(angle < eps) {
        angle = eps;
    } else if(angle > M_PI - eps) {
        angle = M_PI - eps;
    }

    if (angle != theta_) {
        theta_ = angle;
        Q_EMIT thetaChanged(angle);
        wrapped_->getParameter("~view/theta")->set<double>(theta_);
    }
}

void CloudRendererAdapter::setPhi(double angle)
{
    angle = normalizeAngle(angle);
    if (angle != phi_) {
        phi_ = angle;
        Q_EMIT phiChanged(angle);
        wrapped_->getParameter("~view/phi")->set<double>(phi_);
    }
}
