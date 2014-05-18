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
      wrapped_(node), view_(NULL), pixmap_(NULL), fbo_(NULL), drag_(false),
      w_(10), h_(10), point_size_(1),
      phi_(0), theta_(M_PI/2), r_(-10.0),
      list_(0)
{
    node->display_request.connect(boost::bind(&CloudRendererAdapter::display, this));
    node->refresh_request.connect(boost::bind(&CloudRendererAdapter::refresh, this));

    QObject::connect(this, SIGNAL(repaintRequest()), this, SLOT(paintGL()));
    QObject::connect(this, SIGNAL(resizeRequest()), this, SLOT(resize()));
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
    resizeGL(w_, h_);
}

void CloudRendererAdapter::paintGL(bool request)
{
    makeCurrent();
    initializeGL();

    if(fbo_) {
        delete fbo_;
    }

    fbo_ = new QGLFramebufferObject(QSize(w_, h_), QGLFramebufferObject::CombinedDepthStencil);
    fbo_->bind();

    qglClearColor(color_bg_);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    QMatrix4x4 lookat;
    lookat.setToIdentity();
    QVector3D eye(-r_ * std::sin(theta_) * std::cos(phi_),
                  -r_ * std::sin(theta_) * std::sin(phi_),
                  -r_ * std::cos(theta_));
    QVector3D center(0,0,0);
    QVector3D up(0,0,1);
    lookat.lookAt(eye + offset_, center + offset_, up);
    glLoadMatrixd(lookat.data());

    glCallList(list_);

    if(drag_) {
        QVector3D o = center + offset_;

        glPointSize(point_size_ * 5);
        glBegin(GL_POINTS);
        glColor3f(1,1,1);
        glVertex3d(o.x(), o.y(), o.z());
        glEnd();

    }

    QImage img = fbo_->toImage();
    //QImage img = grabFrameBuffer(true);

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
    last_pos_ = event->scenePos();
    drag_ = true;
}

void CloudRendererAdapter::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    drag_ = false;

    wrapped_->getParameter("~size/width")->set<int>(w_);
    wrapped_->getParameter("~size/height")->set<int>(h_);
}

void CloudRendererAdapter::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if(!drag_) {
        return;
    }

    QPointF pos = event->scenePos();
    double dx = pos.x() - last_pos_.x();
    double dy = pos.y() - last_pos_.y();

    double f = 0.01;

    if (event->buttons() & Qt::LeftButton) {
        setTheta(theta_ + f * dy);
        setPhi(phi_ + f * -dx);

    } else if (event->buttons() & Qt::MidButton) {
        w_ = std::max(40, std::min(2000, w_ + (int) dx));
        h_ = std::max(40, std::min(2000, h_ + (int) dy));

        view_->setFixedSize(w_,h_);
        resizeGL(w_, h_);

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

        w_ = wrapped_->param<int>("~size/width");
        h_ = wrapped_->param<int>("~size/height");


        if(w_ != width() || h_ != height()) {
            view_->setFixedSize(w_, h_);
            Q_EMIT resizeRequest();
        }

        Q_EMIT repaintRequest();
    }
}

void CloudRendererAdapter::displayCloud()
{
    boost::apply_visitor (PointCloudMessage::Dispatch<CloudRendererAdapter>(this), wrapped_->message_->value);
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

    if(list_ == 0) {
        list_ = glGenLists(1);
    }

    glNewList(list_,GL_COMPILE);

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
    return QSize(w_, h_);
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
