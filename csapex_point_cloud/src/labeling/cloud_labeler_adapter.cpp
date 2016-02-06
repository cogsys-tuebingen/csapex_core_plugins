/// HEADER
#include "cloud_labeler_adapter.h"

/// PROJECT
#include <csapex/view/utility/register_node_adapter.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/msg/io.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/param/range_parameter.h>

/// SYSTEM
#include <QtOpenGL>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <pcl/for_each_type.h>
#include <pcl/conversions.h>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_NODE_ADAPTER(CloudLabelerAdapter, csapex::CloudLabeler)

CloudLabelerAdapter::CloudLabelerAdapter(NodeHandleWeakPtr worker, NodeBox* parent, std::weak_ptr<CloudLabeler> node)
    : QGLWidget(QGLFormat(QGL::SampleBuffers)), DefaultNodeAdapter(worker, parent),
      wrapped_(node), view_(nullptr), pixmap_(nullptr), fbo_(nullptr), drag_(false), repaint_(true),
      fov_v_(45.0f), near_(0.01f), far_(300.0f),
      w_view_(10), h_view_(10), point_size_(1),
      phi_(0), theta_(M_PI/2), r_(-10.0),
      axes_(false), grid_size_(10), grid_resolution_(1.0), grid_xy_(true), grid_yz_(false), grid_xz_(false),
      list_cloud_(0), list_augmentation_(0),
      label_rect_(false), label_point_(false),
      radius_(0.3)
{
    auto node_ptr = wrapped_.lock();

    trackConnection(node_ptr->display_request.connect(std::bind(&CloudLabelerAdapter::display, this)));
    trackConnection(node_ptr->refresh_request.connect(std::bind(&CloudLabelerAdapter::refresh, this)));
    trackConnection(node_ptr->done_request.connect(std::bind(&CloudLabelerAdapter::done, this)));

    QObject::connect(this, SIGNAL(repaintRequest()), this, SLOT(paintGLImpl()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(resizeRequest()), this, SLOT(resize()), Qt::QueuedConnection);
}

CloudLabelerAdapter::~CloudLabelerAdapter()
{
    delete fbo_;
}

void CloudLabelerAdapter::stop()
{
    DefaultNodeAdapter::stop();
    disconnect();
}

void CloudLabelerAdapter::setupUi(QBoxLayout* layout)
{
    view_ = new QGraphicsView;
    QGraphicsScene* scene = view_->scene();
    if(scene == nullptr) {
        scene = new QGraphicsScene();
        selection_ = scene->addRect(0, 0, 0, 0, QPen(Qt::black));
        view_->setScene(scene);
    }

    view_->setContextMenuPolicy(Qt::PreventContextMenu);
    view_->setMouseTracking(true);

    scene->installEventFilter(this);

    layout->addWidget(view_);

    QObject::connect(this, SIGNAL(displayRequest()), this, SLOT(displayCloud()));

    DefaultNodeAdapter::setupUi(layout);
}



void CloudLabelerAdapter::initializeGL()
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

void CloudLabelerAdapter::resizeGL(int width, int height)
{
    makeCurrent();

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    projection.setToIdentity();
    projection.perspective(fov_v_,(GLfloat)width/(GLfloat)height,near_, far_);
    glLoadMatrixf(projection.data());
    glMatrixMode(GL_MODELVIEW);
}

void CloudLabelerAdapter::resize()
{
    if(w_view_ != view_->width() || h_view_ != view_->height()) {
        view_->setFixedSize(w_view_, h_view_);
    }
    resizeGL(w_out_, h_out_);
}

void CloudLabelerAdapter::paintAugmentation()
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

std::pair<QVector3D, QVector3D>
CloudLabelerAdapter::calculateRay( const QPointF& cursor)
{
    QVector3D view = (look_at_pt - eye).normalized();

    QVector3D h = QVector3D::crossProduct(view, up).normalized();
    QVector3D v = QVector3D::crossProduct(h, view).normalized();

    double vLength = std::tan((fov_v_ * M_PI / 180.0) / 2) * near_;
    double hLength = vLength * (w_view_ / h_view_);

    v *= vLength;
    h *= hLength;

    double x = cursor.x();
    double y = h_view_ - cursor.y();

    x -= w_view_ / 2;
    y -= h_view_ / 2;

    x /= (w_view_ / 2);
    y /= (h_view_ / 2);

    QVector3D pos = eye + view * near_ + (h*x + v*y);
    QVector3D dir = (pos - eye).normalized();

    std::pair<QVector3D, QVector3D> ray { eye + dir * far_ / 4.0, dir };
    return ray;
}

void CloudLabelerAdapter::paintGLImpl(bool request)
{
    auto n = wrapped_.lock();
    if(!n) {
        return;
    }

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
    modelview.setToIdentity();
    eye = QVector3D(-r_ * std::sin(theta_) * std::cos(phi_),
                    -r_ * std::sin(theta_) * std::sin(phi_),
                    -r_ * std::cos(theta_))
            + offset_;
    QVector3D center(0,0,0);
    up = QVector3D(0,0,1);

    look_at_pt = center + offset_;

    modelview.lookAt(eye, look_at_pt, up);
    glLoadMatrixf(modelview.data());

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

    // cursor
    glDisable(GL_CULL_FACE);

    glColor3f(0.0, 0.0, 0.0);

    glBegin(GL_QUADS);
    glVertex3d(selection_a_.first.x(), selection_a_.first.y(), selection_a_.first.z());
    glVertex3d(selection_b_.first.x(), selection_b_.first.y(), selection_b_.first.z());
    glVertex3d(selection_d_.first.x(), selection_d_.first.y(), selection_d_.first.z());
    glVertex3d(selection_c_.first.x(), selection_c_.first.y(), selection_c_.first.z());

    glEnd();

    glBegin(GL_TRIANGLES);
    glVertex3d(selection_a_.first.x(), selection_a_.first.y(), selection_a_.first.z());
    glVertex3d(selection_b_.first.x(), selection_b_.first.y(), selection_b_.first.z());
    glVertex3d(selection_eye_.x(), selection_eye_.y(), selection_eye_.z());

    glVertex3d(selection_b_.first.x(), selection_b_.first.y(), selection_b_.first.z());
    glVertex3d(selection_d_.first.x(), selection_d_.first.y(), selection_d_.first.z());
    glVertex3d(selection_eye_.x(), selection_eye_.y(), selection_eye_.z());

    glVertex3d(selection_d_.first.x(), selection_d_.first.y(), selection_d_.first.z());
    glVertex3d(selection_c_.first.x(), selection_c_.first.y(), selection_c_.first.z());
    glVertex3d(selection_eye_.x(), selection_eye_.y(), selection_eye_.z());

    glVertex3d(selection_c_.first.x(), selection_c_.first.y(), selection_c_.first.z());
    glVertex3d(selection_a_.first.x(), selection_a_.first.y(), selection_a_.first.z());
    glVertex3d(selection_eye_.x(), selection_eye_.y(), selection_eye_.z());

    glEnd();

    glPushMatrix();
    glTranslated(selection_3d_cursor_.x(), selection_3d_cursor_.y(), selection_3d_cursor_.z());
    drawSphere(radius_, 16, 16);
    glPopMatrix();

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
    pixmap_->setZValue(0);
    selection_->setZValue(1);
    view_->blockSignals(false);
}

void CloudLabelerAdapter::drawSphere(double radius, int lats, int longs) {
    for(int i = 0; i <= lats; i++) {
        double lat0 = M_PI * (-0.5 + (double) (i - 1) / lats);
        double z0  = sin(lat0);
        double zr0 =  cos(lat0);

        double lat1 = M_PI * (-0.5 + (double) i / lats);
        double z1 = sin(lat1);
        double zr1 = cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for(int j = 0; j <= longs; j++) {
            double lng = 2 * M_PI * (double) (j - 1) / longs;
            double x = cos(lng);
            double y = sin(lng);

            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0);
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1);
        }
        glEnd();
    }
}

void CloudLabelerAdapter::labelPoint()
{
    NodeHandlePtr node_handle = node_.lock();
    if(!node_handle) {
        return;
    }
    auto node = node_handle->getNode().lock();
    if(!node) {
        return;
    }

    int label = node->readParameter<int>("label");
    radius_ = node->readParameter<double>("radius");


    for(pcl::PointXYZL& pt : labeled_->points) {
        QVector3D v(pt.x, pt.y, pt.z);
        if(v.distanceToPoint(selection_3d_cursor_) < radius_) {
            pt.label = label;
        }
    }

    drawPoints();

    repaintRequest();
}

void CloudLabelerAdapter::labelArea()
{

    NodeHandlePtr node_handle = node_.lock();
    if(!node_handle) {
        return;
    }
    auto node = node_handle->getNode().lock();
    if(!node) {
        return;
    }
    int label = node->readParameter<int>("label");


    double minx = std::min(cursor_label_start_.x(), cursor_label_end_.x());
    double maxx = std::max(cursor_label_start_.x(), cursor_label_end_.x());
    double miny = std::min(cursor_label_start_.y(), cursor_label_end_.y());
    double maxy = std::max(cursor_label_start_.y(), cursor_label_end_.y());

    QPointF tl(minx, miny);
    QPointF tr(maxx, miny);
    QPointF bl(minx, maxy);
    QPointF br(maxx, maxy);

    selection_eye_ = eye;

    selection_a_ = calculateRay(tl);
    selection_b_ = calculateRay(tr);
    selection_c_ = calculateRay(bl);
    selection_d_ = calculateRay(br);

    QVector3D n_ab = QVector3D::crossProduct(selection_a_.second, selection_b_.second);
    QVector3D n_bd = QVector3D::crossProduct(selection_b_.second, selection_d_.second);;
    QVector3D n_dc = QVector3D::crossProduct(selection_d_.second, selection_c_.second);;
    QVector3D n_ca = QVector3D::crossProduct(selection_c_.second, selection_a_.second);;

    QVector3D normals[] {
        n_ab, n_bd, n_dc, n_ca
    };

    for(pcl::PointXYZL& pt : labeled_->points) {
        QVector3D v(pt.x, pt.y, pt.z);
        bool inside = true;
        for(const QVector3D& n : normals) {
            double dot = QVector3D::dotProduct(n, v - eye);
            if(dot < 0) {
                inside = false;
                break;
            }
        }
        if(inside) {
            pt.label = label;
        }
    }

    drawPoints();

    repaintRequest();
}

bool CloudLabelerAdapter::eventFilter(QObject * o, QEvent * e)
{
    auto n = node_.lock();
    if(!n) {
        return false;
    }

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

    case QEvent::KeyPress: {
        QKeyEvent* ke = dynamic_cast<QKeyEvent*>(e);

        int key = ke->key();

        if(Qt::Key_0 <= key && key <= Qt::Key_9) {
            updateLabel(ke->key() - Qt::Key_0);
        } else if(key == Qt::Key_Enter) {
            done();
        } else if(key == Qt::Key_Escape) {
            updateLabel(0);
        }

        break;
    }
    default:
        break;
    }

    return false;
}

void CloudLabelerAdapter::mousePressEventImpl(QGraphicsSceneMouseEvent *event)
{
    last_pos_ = event->screenPos();

    cursor_ = event->scenePos();

    bool shift = Qt::ShiftModifier & QApplication::keyboardModifiers();
    bool ctrl = Qt::ControlModifier & QApplication::keyboardModifiers();
    if(!shift) {

        if(ctrl) {
            label_rect_ = true;
            cursor_label_start_ = cursor_;
            event->accept();
            selection_->setRect(0, 0, 0, 0);

        } else {
            label_point_ = true;
            labelPoint();
            event->accept();
        }


    } else {
        drag_ = true;
        event->accept();
    }
}

void CloudLabelerAdapter::mouseReleaseEventImpl(QGraphicsSceneMouseEvent *event)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    cursor_ = event->scenePos();

    if(label_rect_) {
        cursor_label_end_ = cursor_;
        labelArea();

        selection_->setRect(0, 0, 0, 0);
    }

    label_rect_ = false;
    label_point_ = false;
    drag_ = false;

    node->getParameter("~size/width")->set<int>(w_view_);
    node->getParameter("~size/height")->set<int>(h_view_);

    if(size_sync_) {
        w_out_ = w_view_;
        h_out_ = h_view_;
        node->getParameter("~size/out/width")->set<int>(w_out_);
        node->getParameter("~size/out/height")->set<int>(h_out_);
    }
    event->accept();
}

void CloudLabelerAdapter::mouseMoveEventImpl(QGraphicsSceneMouseEvent *event)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    cursor_ = event->scenePos();

    auto ray = calculateRay(cursor_);

    double closest_dist = std::numeric_limits<double>::infinity();

    for(pcl::PointXYZL& pt : labeled_->points) {
        QVector3D v(pt.x, pt.y, pt.z);
        QVector3D ve = v - eye;
        double dist = v.distanceToLine(eye, ray.second);
        if(dist < 0.05) {
            if(ve.length() < closest_dist) {
                closest_dist = ve.length();
            }
        }
    }

    selection_3d_cursor_ = eye + ray.second * closest_dist;

    if(label_point_) {
        labelPoint();
        event->accept();
    }
    if(label_rect_) {
        selection_->setRect(QRectF(std::min(cursor_label_start_.x(), cursor_.x()),
                                   std::min(cursor_label_start_.y(), cursor_.y()),
                                   std::abs(cursor_label_start_.x() - cursor_.x()),
                                   std::abs(cursor_label_start_.y() - cursor_.y())));
        event->accept();
    }

    if(!label_rect_ && drag_) {
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


            node->getParameter("~view/dx")->set<double>(offset_.x());
            node->getParameter("~view/dy")->set<double>(offset_.y());
            node->getParameter("~view/dz")->set<double>(offset_.z());

        }
        last_pos_ = pos;
    }
    paintGLImpl(false);
}

void CloudLabelerAdapter::wheelEventImpl(QGraphicsSceneWheelEvent *event)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    bool shift = Qt::ShiftModifier & QApplication::keyboardModifiers();
    if(!shift) {
        param::RangeParameterPtr p = std::dynamic_pointer_cast<param::RangeParameter>(node->getParameter("radius"));
        radius_ += event->delta() * 0.001;
        radius_ = std::min(p->max<double>(), std::max(p->min<double>(), radius_));
        p->set<double>(radius_);
    } else {
        r_ += event->delta() * -0.0025;
        node->getParameter("~view/r")->set<double>(r_);
    }
    event->accept();
    paintGLImpl(false);
}


void CloudLabelerAdapter::display()
{
    Q_EMIT displayRequest();
}

void CloudLabelerAdapter::refresh()
{
    NodeHandlePtr node_handle = node_.lock();
    if(!node_handle) {
        return;
    }
    auto node = node_handle->getNode().lock();
    if(!node) {
        return;
    }
    radius_ = node->readParameter<double>("radius");

    if(!drag_){
        {
            const std::vector<int>& c = node->readParameter<std::vector<int> >("color/background");
            color_bg_ = QColor::fromRgb(c[0], c[1], c[2]);
        }
        {
            const std::vector<int>& c = node->readParameter<std::vector<int> >("color/grid");
            color_grid_ = QColor::fromRgb(c[0], c[1], c[2]);
        }

        r_ = node->readParameter<double>("~view/r");
        theta_ = node->readParameter<double>("~view/theta");
        phi_ = node->readParameter<double>("~view/phi");

        double dx = node->readParameter<double>("~view/dx");
        double dy = node->readParameter<double>("~view/dy");
        double dz = node->readParameter<double>("~view/dz");

        offset_ = QVector3D(dx,dy,dz);
        point_size_ = node->readParameter<double>("point/size");

        size_sync_ = node->readParameter<bool>("~size/out/sync");
        w_view_ = node->readParameter<int>("~size/width");
        h_view_ = node->readParameter<int>("~size/height");

        if(size_sync_) {
            w_out_ = w_view_;
            h_out_ = h_view_;
        } else {
            w_out_ = node->readParameter<int>("~size/out/width");
            h_out_ = node->readParameter<int>("~size/out/height");
        }

        axes_ = node->readParameter<bool>("show axes");

        grid_size_ = node->readParameter<int>("~grid/size");
        grid_resolution_ = node->readParameter<double>("~grid/resolution");
        grid_xy_ = node->readParameter<bool>("~grid/xy");
        grid_yz_ = node->readParameter<bool>("~grid/yz");
        grid_xz_ = node->readParameter<bool>("~grid/xz");

        repaint_ = true;

        Q_EMIT resizeRequest();

        Q_EMIT repaintRequest();
    }
}

void CloudLabelerAdapter::done()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    node->setResult(labeled_);
}

void CloudLabelerAdapter::displayCloud()
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    auto copy = node->getMessage();
    boost::apply_visitor (PointCloudMessage::Dispatch<CloudLabelerAdapter>(this, copy), copy->value);
}

template <class PointT>
void CloudLabelerAdapter::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }
    labeled_.reset(new pcl::PointCloud<pcl::PointXYZL>);
    labeled_->header = cloud->header;
    labeled_->points.resize(cloud->points.size());
    pcl::PointXYZL* pt_label = &labeled_->points.front();
    for(const PointT& pt : cloud->points) {
        pt_label->x = pt.x;
        pt_label->y = pt.y;
        pt_label->z = pt.z;
        pt_label->label = 0;

        ++pt_label;
    }


    makeCurrent();

    if(list_cloud_ == 0) {
        list_cloud_ = glGenLists(1);
    }

    drawPoints();

    paintGLImpl();
}

void CloudLabelerAdapter::drawPoints()
{
    makeCurrent();

    glNewList(list_cloud_,GL_COMPILE);

    glPointSize(point_size_);

    glBegin(GL_POINTS);

    glColor3d(0.0, 0.0, 0.0);

    for(auto it = labeled_->points.begin(); it != labeled_->points.end(); ++it) {
        const pcl::PointXYZL& pt = *it;
        if(pt.label == 0) {
            glColor3d(0.0, 0.0, 0.0);
            glVertex3d(pt.x, pt.y, pt.z);
        }
    }

    glEnd();

    glPointSize(point_size_/2.0);

    glBegin(GL_POINTS);

    for(auto it = labeled_->points.begin(); it != labeled_->points.end(); ++it) {
        const pcl::PointXYZL& pt = *it;

        if(pt.label > 0) {
            double r = 0.0, g = 0.0, b = 0.0;
            color::fromCount(pt.label, r, g, b);
            glColor3d(r/255., g/255., b/255.);

            glVertex3d(pt.x, pt.y, pt.z);
        }

    }

    glEnd();

    glEndList();
}


QSize CloudLabelerAdapter::minimumSizeHint() const
{
    return QSize(10, 10);
}

QSize CloudLabelerAdapter::sizeHint() const
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

void CloudLabelerAdapter::setTheta(double angle)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }

    double eps = 1e-3;
    if(angle < eps) {
        angle = eps;
    } else if(angle > M_PI - eps) {
        angle = M_PI - eps;
    }

    if (angle != theta_) {
        theta_ = angle;
        Q_EMIT thetaChanged(angle);
        node->getParameter("~view/theta")->set<double>(theta_);
    }
}

void CloudLabelerAdapter::setPhi(double angle)
{
    auto node = wrapped_.lock();
    if(!node) {
        return;
    }


    angle = normalizeAngle(angle);
    if (angle != phi_) {
        phi_ = angle;
        Q_EMIT phiChanged(angle);
        node->getParameter("~view/phi")->set<double>(phi_);
    }
}
void CloudLabelerAdapter::updateLabel(int label)
{
    NodeHandlePtr node_handle = node_.lock();
    if(node_handle) {
        auto node = node_handle->getNode().lock();
        if(node) {
            node->getParameter("label")->set(label);
        }
    }
}
/// MOC
#include "moc_cloud_labeler_adapter.cpp"
