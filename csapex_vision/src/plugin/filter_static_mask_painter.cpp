/// HEADER
#include "filter_static_mask_painter.h"

/// SYSTEM
#include <QPixmap>
#include <QGraphicsSceneEvent>
#include <utils_qt/QtCvImageConverter.h>
#include <QKeyEvent>
#include <QPushButton>
#include <QBoxLayout>

using namespace csapex;

StaticMaskPainter::StaticMaskPainter(const cv::Mat& mask)
    : dragging(false)
{
    mask.copyTo(mask_);
}

StaticMaskPainter::~StaticMaskPainter()
{
}

void StaticMaskPainter::run()
{
    QDialog* modal = new QDialog();
    modal->setModal(true);
    modal->resize(400, 400);

    view = new QGraphicsView;
    scene = new QGraphicsScene;
    view->setScene(scene);

    scene->installEventFilter(this);

    buttons = new QDialogButtonBox;
    buttons->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel | QDialogButtonBox::Reset);

    QObject::connect(buttons, SIGNAL(accepted()), modal, SLOT(accept()));
    QObject::connect(buttons, SIGNAL(rejected()), modal, SLOT(reject()));
    QObject::connect(buttons, SIGNAL(clicked(QAbstractButton*)), this, SLOT(buttonClicked(QAbstractButton*)));

    QBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(view);
    layout->addWidget(buttons);

    modal->setLayout(layout);

    int result = modal->exec();
    if(result == QDialog::Accepted) {
        Q_EMIT new_mask(mask_);
    }

    delete modal;
}

void StaticMaskPainter::buttonClicked(QAbstractButton* btn)
{
    if (buttons->button(QDialogButtonBox::Reset) == btn) {
        mask_.release();
        mask_backup.release();
    }
}


bool StaticMaskPainter::eventFilter(QObject* obj, QEvent* event)
{
    switch(event->type()) {
    case QEvent::GraphicsSceneMousePress: {
        QGraphicsSceneMouseEvent* me = static_cast<QGraphicsSceneMouseEvent*>(event);

        if(dragging) {
            dragging = false;
            mask_backup.copyTo(mask_);
            break;

        } else  {
            mask_.copyTo(mask_backup);

            dragging = true;

            start_drag = me->scenePos();
            start_btn = me->button();
        }
        break;
    }
    case QEvent::GraphicsSceneMouseMove: {
        if(dragging) {
            QGraphicsSceneMouseEvent* me = static_cast<QGraphicsSceneMouseEvent*>(event);

            QPointF current = me->scenePos();
            cv::Rect rec(cv::Point(start_drag.x(), start_drag.y()),
                         cv::Point(current.x(), current.y()));
            mask_backup.copyTo(mask_);
            cv::rectangle(mask_, rec, cv::Scalar::all(0), 2);
        }
        break;
    }

    case QEvent::GraphicsSceneMouseRelease: {
        QGraphicsSceneMouseEvent* me = static_cast<QGraphicsSceneMouseEvent*>(event);

        mask_backup.copyTo(mask_);

        if(me->button() == start_btn) {

            QPointF stop_drag = me->scenePos();

            cv::Rect rec(cv::Point(start_drag.x(), start_drag.y()),
                         cv::Point(stop_drag.x(), stop_drag.y()));

            cv::rectangle(mask_, rec, cv::Scalar::all(0), CV_FILLED);
        }

        dragging = false;
        break;
    }

    default:
        break;
    }

    return QObject::eventFilter(obj, event);
}

void StaticMaskPainter::setMask(cv::Mat mask)
{
    if(!mask.empty()) {
        mask.copyTo(this->mask_);
    }
}

void StaticMaskPainter::input(cv::Mat img)
{
    cv::Mat masked;
    if(mask_.empty()) {
        mask_ = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar(255));
    }
    img.copyTo(masked, mask_);

    QSharedPointer<QImage> image = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(masked);
    QGraphicsScene* scene = view->scene();
    scene->clear();
    scene->setSceneRect(0, 0, img.cols, img.rows);
    scene->addPixmap(QPixmap::fromImage(*image));

    view->fitInView(scene->itemsBoundingRect() ,Qt::KeepAspectRatio);
}
