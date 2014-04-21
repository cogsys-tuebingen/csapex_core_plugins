#ifndef FILTER_STATIC_MASK_PAINTER_H
#define FILTER_STATIC_MASK_PAINTER_H

/// SYSTEM
#include <QDialog>
#include <QGraphicsView>
#include <QDialogButtonBox>
#include <opencv2/opencv.hpp>

namespace csapex
{

class StaticMaskPainter : public QObject
{
    Q_OBJECT

public:
    StaticMaskPainter(const cv::Mat& mask_);
    virtual ~StaticMaskPainter();
    void run();

    bool eventFilter(QObject* obj, QEvent* event);

public Q_SLOTS:
    void input(cv::Mat img);
    void setMask(cv::Mat mask_);

    void buttonClicked(QAbstractButton*);

Q_SIGNALS:
    void new_mask(cv::Mat);

private:
    QGraphicsView* view;
    QGraphicsScene* scene;
    QDialogButtonBox* buttons;

    cv::Mat mask_;
    cv::Mat mask_backup;

    QPointF start_drag;
    Qt::MouseButton start_btn;
    bool dragging;
};

}

#endif // FILTER_STATIC_MASK_PAINTER_H
