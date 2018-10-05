/// HEADER
#include <csapex_core_plugins/image_widget.h>

/// SYSTEM
#include <QBoxLayout>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneEvent>
#include <QPainter>
#include <QPushButton>
#include <QResizeEvent>

using namespace csapex;

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent), size(100, 100), manual_resize_(false)
{
}

void ImageWidget::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);

    if (pix.isNull())
        return;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QSize pixSize = pix.size();
    pixSize.scale(event->rect().size(), Qt::KeepAspectRatio);

    QPixmap scaledPix = pix.scaled(pixSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    int x = (width() - scaledPix.width()) / 2;
    int y = (height() - scaledPix.height()) / 2;

    painter.drawPixmap(QPoint(x, y), scaledPix);
}

void ImageWidget::resizeEvent(QResizeEvent* re)
{
    if (manual_resize_) {
        setSize(re->size());
    }
}

const QPixmap* ImageWidget::pixmap() const
{
    return &pix;
}

void ImageWidget::setPixmap(const QPixmap& pixmap)
{
    pix = pixmap;
}

void ImageWidget::setManualResize(bool manual)
{
    manual_resize_ = manual;
    updateGeometry();
}

void ImageWidget::setSize(const QSize& s)
{
    size = s;
    updateGeometry();
}
void ImageWidget::setSize(int w, int h)
{
    setSize(QSize(w, h));
}

QSize ImageWidget::sizeHint() const
{
    return size;
}
QSize ImageWidget::minimumSizeHint() const
{
    if (manual_resize_) {
        return QSize(10, 10);
    } else {
        return size;
    }
}
