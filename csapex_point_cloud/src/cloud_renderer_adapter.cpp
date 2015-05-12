/// HEADER
#include "cloud_renderer_adapter.h"

/// PROJECT
#include <csapex/utility/register_node_adapter.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/q_signal_relay.h>
#include <csapex/msg/io.h>

/// SYSTEM
#include <QtOpenGL>
#include <utils_qt/QtCvImageConverter.h>
#include <pcl/for_each_type.h>
#include <pcl/conversions.h>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_NODE_ADAPTER(CloudRendererAdapter, csapex::CloudRenderer)

CloudRendererAdapter::CloudRendererAdapter(NodeWorker* worker, CloudRenderer *node, WidgetController* widget_ctrl)
    : QGLWidget(QGLFormat(QGL::SampleBuffers)), DefaultNodeAdapter(worker, widget_ctrl),
      wrapped_(node), view_(nullptr), pixmap_(nullptr), fbo_(nullptr), drag_(false), repaint_(true),
      w_view_(10), h_view_(10), point_size_(1),
      phi_(0), theta_(M_PI/2), r_(-10.0),
      axes_(false), grid_size_(10), grid_resolution_(1.0), grid_xy_(true), grid_yz_(false), grid_xz_(false),
      list_cloud_(0), list_augmentation_(0)
{
    node->display_request.connect(std::bind(&CloudRendererAdapter::display, this));
    node->refresh_request.connect(std::bind(&CloudRendererAdapter::refresh, this));

    QObject::connect(this, SIGNAL(repaintRequest()), this, SLOT(paintGLImpl()), Qt::QueuedConnection);
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
    if(scene == nullptr) {
        scene = new QGraphicsScene();
        //scene->addWidget(this);
        view_->setScene(scene);
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
    glLoadMatrixf(projection.data());
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

void CloudRendererAdapter::paintGLImpl(bool request)
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
    glLoadMatrixf(lookat.data());

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

    if(pixmap_ == nullptr) {
        pixmap_ = view_->scene()->addPixmap(QPixmap::fromImage(img));
    } else {
        pixmap_->setPixmap(QPixmap::fromImage(img));
    }

    view_->blockSignals(true);
    view_->scene()->setSceneRect(img.rect());
    view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
    view_->blockSignals(false);
    //    view_->scene()->update();

    if(msg::isConnected(wrapped_->output_) && request){
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
        mousePressEventImpl(me);
        return true;
    case QEvent::GraphicsSceneMouseRelease:
        mouseReleaseEventImpl(me);
        return true;
    case QEvent::GraphicsSceneMouseMove:
        mouseMoveEventImpl(me);
        return true;
    case QEvent::GraphicsSceneWheel:
        wheelEventImpl(dynamic_cast<QGraphicsSceneWheelEvent*>(e));
        return true;

    default:
        break;
    }

    return false;
}

void CloudRendererAdapter::mousePressEventImpl(QGraphicsSceneMouseEvent *event)
{
    last_pos_ = event->screenPos();
    drag_ = true;
    event->accept();
}

void CloudRendererAdapter::mouseReleaseEventImpl(QGraphicsSceneMouseEvent *event)
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

void CloudRendererAdapter::mouseMoveEventImpl(QGraphicsSceneMouseEvent *event)
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
    paintGLImpl(false);
}

void CloudRendererAdapter::wheelEventImpl(QGraphicsSceneWheelEvent *event)
{
    event->accept();

    r_ += event->delta() * -0.0025;
    wrapped_->getParameter("~view/r")->set<double>(r_);
    paintGLImpl(false);

}


void CloudRendererAdapter::display()
{
    Q_EMIT displayRequest();
}

void CloudRendererAdapter::refresh()
{
    if(!drag_){
        {
            const std::vector<int>& c = wrapped_->readParameter<std::vector<int> >("color/background");
            color_bg_ = QColor::fromRgb(c[0], c[1], c[2]);
        }
        {
            const std::vector<int>& c = wrapped_->readParameter<std::vector<int> >("color/grid");
            color_grid_ = QColor::fromRgb(c[0], c[1], c[2]);
        }
        {
            const std::vector<int>& c = wrapped_->readParameter<std::vector<int> >("color/gradient/start");
            color_grad_start_ = QVector3D(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0);
        }
        {
            const std::vector<int>& c = wrapped_->readParameter<std::vector<int> >("color/gradient/end");
            color_grad_end_ = QVector3D(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0);
        }
        r_ = wrapped_->readParameter<double>("~view/r");
        theta_ = wrapped_->readParameter<double>("~view/theta");
        phi_ = wrapped_->readParameter<double>("~view/phi");

        double dx = wrapped_->readParameter<double>("~view/dx");
        double dy = wrapped_->readParameter<double>("~view/dy");
        double dz = wrapped_->readParameter<double>("~view/dz");

        offset_ = QVector3D(dx,dy,dz);
        point_size_ = wrapped_->readParameter<double>("point/size");

        size_sync_ = wrapped_->readParameter<bool>("~size/out/sync");
        w_view_ = wrapped_->readParameter<int>("~size/width");
        h_view_ = wrapped_->readParameter<int>("~size/height");

        if(size_sync_) {
            w_out_ = w_view_;
            h_out_ = h_view_;
        } else {
            w_out_ = wrapped_->readParameter<int>("~size/out/width");
            h_out_ = wrapped_->readParameter<int>("~size/out/height");
        }

        axes_ = wrapped_->readParameter<bool>("show axes");

        grid_size_ = wrapped_->readParameter<int>("~grid/size");
        grid_resolution_ = wrapped_->readParameter<double>("~grid/resolution");
        grid_xy_ = wrapped_->readParameter<bool>("~grid/xy");
        grid_yz_ = wrapped_->readParameter<bool>("~grid/yz");
        grid_xz_ = wrapped_->readParameter<bool>("~grid/xz");

        repaint_ = true;

        Q_EMIT resizeRequest();

        Q_EMIT repaintRequest();
    }
}

void CloudRendererAdapter::displayCloud()
{
    auto copy = wrapped_->message_;
    boost::apply_visitor (PointCloudMessage::Dispatch<CloudRendererAdapter>(this, copy), copy->value);
}

namespace
{

enum Component
{
    X, Y, Z, I
};


template <class PointT, int Component>
struct Access
{
};

template <class PointT>
struct Access<PointT, X>
{
    static double access(typename pcl::PointCloud<PointT>::const_iterator it)
    {
        return it->x;
    }
};

template <class PointT>
struct Access<PointT, Y>
{
    static double access(typename pcl::PointCloud<PointT>::const_iterator it)
    {
        return it->y;
    }
};

template <class PointT>
struct Access<PointT, Z>
{
    static double access(typename pcl::PointCloud<PointT>::const_iterator it)
    {
        return it->z;
    }
};

template <class PointT>
struct Access<PointT, I>
{
    static double access(typename pcl::PointCloud<PointT>::const_iterator it)
    {
        return 0;
    }
};

template <>
struct Access<pcl::PointXYZI, I>
{
    static double access(typename pcl::PointCloud<pcl::PointXYZI>::const_iterator it)
    {
        return it->intensity;
    }
};

template <class PointT, int Component>
struct Util
{
    static void findExtrema(typename pcl::PointCloud<PointT>::ConstPtr cloud, double& min, double& max)
    {
        min = std::numeric_limits<double>::max();
        max = std::numeric_limits<double>::min();
        for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            if(Access<PointT, Component>::access(it) < min) {
                min = Access<PointT, Component>::access(it);
            }
            if(Access<PointT, Component>::access(it) > max) {
                max = Access<PointT, Component>::access(it);
            }
        }
    }
};

// adapted from RVIZ
template <typename Color>
void getRainbowColor(float value, Color& color)
{
    value = std::max(std::min(value, 1.0f), 0.0f);

    float h = value * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if ( !(i&1) ) f = 1 - f; // if i is even
    float n = 1 - f;

    if (i <= 1) color.setX(n), color.setY(0), color.setZ(1);
    else if (i == 2) color.setX(0), color.setY(n), color.setZ(1);
    else if (i == 3) color.setX(0), color.setY(1), color.setZ(n);
    else if (i == 4) color.setX(n), color.setY(1), color.setZ(0);
    else if (i >= 5) color.setX(1), color.setY(n), color.setZ(0);
}

template <class PointT, int Component>
struct RendererGradient
{
    static void render(typename pcl::PointCloud<PointT>::ConstPtr cloud, bool rainbow, const QVector3D& color_grad_start, const QVector3D& color_grad_end)
    {
        if(rainbow) {
            renderRainbow(cloud);
        } else {
            renderGradient(cloud, color_grad_start, color_grad_end);
        }
    }
    static void renderGradient(typename pcl::PointCloud<PointT>::ConstPtr cloud, const QVector3D& color_grad_start, const QVector3D& color_grad_end)
    {
        double min, max;
        Util<PointT, Component>::findExtrema(cloud, min, max);

        for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            const PointT& pt = *it;

            double v = Access<PointT, Component>::access(it);
            double f = (v - min) / (max - min);
            QVector3D c = f * color_grad_start + (1.0-f) * color_grad_end;

            glColor3d(c.x(), c.y(), c.z());

            glVertex3d(pt.x, pt.y, pt.z);
        }
    }
    static void renderRainbow(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        double min, max;
        Util<PointT, Component>::findExtrema(cloud, min, max);

        for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            const PointT& pt = *it;

            double v = Access<PointT, Component>::access(it);
            double f = (v - min) / (max - min);
            QVector3D c;
            getRainbowColor(f, c);

            glColor3d(c.x(), c.y(), c.z());

            glVertex3d(pt.x, pt.y, pt.z);
        }
    }
};

template <class PointT, int Component>
struct Renderer
{
    static void render(typename pcl::PointCloud<PointT>::ConstPtr cloud, bool rainbow, const QVector3D& color_grad_start, const QVector3D& color_grad_end)
    {
        RendererGradient<PointT, Component>::render(cloud, rainbow, color_grad_start, color_grad_end);
    }
};

template <int Component>
struct Renderer<pcl::PointXYZRGB, Component>
{
    static void render(typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, bool, const QVector3D&, const QVector3D&)
    {
        for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            const pcl::PointXYZRGB& pt = *it;

            glColor3d(pt.r / 255.0, pt.g / 255.0, pt.b / 255.0);

            glVertex3d(pt.x, pt.y, pt.z);
        }
    }
};
}

template <class PointT>
void CloudRendererAdapter::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    makeCurrent();

    if(list_cloud_ == 0) {
        list_cloud_ = glGenLists(1);
    }

    glNewList(list_cloud_,GL_COMPILE);

    glPointSize(point_size_);

    glBegin(GL_POINTS);

    std::string component = wrapped_->readParameter<std::string>("color/field");

    bool rainbow = wrapped_->readParameter<bool>("color/rainbow");

    if(wrapped_->readParameter<bool>("color/force gradient")) {
        if(component == "x") {
            RendererGradient<PointT, X>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        } else if(component == "y") {
            RendererGradient<PointT, Y>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        } else if(component == "z") {
            RendererGradient<PointT, Z>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        } else if(component == "i") {
            RendererGradient<PointT, I>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        }

    } else {
        if(component == "x") {
            Renderer<PointT, X>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        } else if(component == "y") {
            Renderer<PointT, Y>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        } else if(component == "z") {
            Renderer<PointT, Z>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        } else if(component == "i") {
            Renderer<PointT, I>::render(cloud, rainbow, color_grad_start_, color_grad_end_);
        }
    }

    glEnd();

    glEndList();

    paintGLImpl();
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
/// MOC
#include "moc_cloud_renderer_adapter.cpp"
