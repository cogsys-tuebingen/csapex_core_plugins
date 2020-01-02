#ifndef IMAGE_WIDGET_H
#define IMAGE_WIDGET_H

/// SYSTEM
#include <QGraphicsView>
#include <QLabel>

namespace csapex
{
class ImageWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImageWidget(QWidget* parent = 0);
    const QPixmap* pixmap() const;

    void setManualResize(bool manual);

    void setSize(const QSize& size);
    void setSize(int w, int h);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

public Q_SLOTS:
    void setPixmap(const QPixmap&);

protected:
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;

private:
    QPixmap pix;
    QSize size;

    bool manual_resize_;
};

}  // namespace csapex

#endif  // IMAGE_WIDGET_H
