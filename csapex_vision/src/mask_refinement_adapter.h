#ifndef MASK_REFINEMENT_ADAPTER_H
#define MASK_REFINEMENT_ADAPTER_H

/// PROJECT
#include <csapex/model/generic_state.h>
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include "mask_refinement.h"

/// SYSTEM
#include <QGraphicsView>
#include <yaml-cpp/yaml.h>

namespace csapex
{
class MaskRefinementAdapter : public QObject, public DefaultNodeAdapter
{
    Q_OBJECT

public:
    MaskRefinementAdapter(NodeFacadeImplementationPtr worker, NodeBox* parent, std::weak_ptr<MaskRefinement> node);
    ~MaskRefinementAdapter();

    virtual GenericStatePtr getState() const;
    virtual void setParameterState(GenericStatePtr memento);

    virtual void setupUi(QBoxLayout* layout);

Q_SIGNALS:
    void nextRequest();
    void updateBrushRequest();
    void inputRequest(QImage mask, QImage masked);

public Q_SLOTS:
    void next();
    void setMask(QImage mask, QImage masked);
    void updateBrush();

protected:
    bool eventFilter(QObject* o, QEvent* e);

    struct State : public GenericState
    {
        int width;
        int height;

        State() : width(300), height(300)
        {
        }

        virtual void writeYaml(YAML::Node& out) const
        {
            out["width"] = width;
            out["height"] = height;
        }

        virtual void readYaml(const YAML::Node& node)
        {
            width = node["width"].as<int>();
            height = node["height"].as<int>();
            ;
        }
    };

private:
    void fitView();
    void updateCursor(const QPointF& pos);
    void draw(const QPointF& pos, Qt::GlobalColor color);
    void maskImage();

protected:
    std::weak_ptr<MaskRefinement> wrapped_;

private:
    State state;

    QImage mask_;
    QPixmap mask_pm_;
    QGraphicsPixmapItem* mask_pixmap_;

    QImage img_;

    QPixmap masked_red_pm_;
    QGraphicsPixmapItem* masked_red_pixmap_;

    QPixmap masked_bw_pm_;
    QGraphicsPixmapItem* masked_bw_pixmap_;

    QGraphicsView* view_;
    QGraphicsEllipseItem* cursor_;

    QPoint middle_last_pos_;

    bool refresh_;

    int brush_size_;
};

}  // namespace csapex

#endif  // MASK_REFINEMENT_ADAPTER_H
