#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex/csapex_fwd.h>
#include <QSlider>
#include <QComboBox>
#include "object_homography.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <config/reconfigurable.h>
#include <utils/extractor.h>

/// SYSTEM
#include <QPushButton>

namespace csapex
{

class ObjectDetection : public BoxedObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<ObjectDetection> Ptr;

public:
    ObjectDetection();

    /// insert GUI elements
    virtual void fill(QBoxLayout* layout);
    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private Q_SLOTS:
    /// callback when a message arrives
    void messageArrived(ConnectorIn* source);
    virtual void updateDynamicGui(QBoxLayout *layout);
    void updateDetector(int slot);
    void updateExtractor(int slot);

private Q_SLOTS:
    void update();
    void updatedet();
    void updateModel();

    /// callback from UI button
    void buttonPressed();
    void updateSliders();

private:
    std::vector<QObject*> callbacks_detector;
    std::vector<QObject*> callbacks_extractor;

    QMutex extractor_mutex;
    Extractor::Ptr extractor;
    Extractor::Ptr descriptor;

    //Methods
    /// connectors that were added to the parent box
    ConnectorIn* in_a_;
    ConnectorIn* in_b_;
    ConnectorIn* in_c_;
    ConnectorIn* in_d_;
    ConnectorOut* out_;

    /// Layout
    QComboBox                   *detectorbox_;
    QComboBox                   *extractorbox_;
    QComboBox                   *matcherbox_;
    QFrame                      *opt;
    QFrame                      *opt1;

    /// flags to remember, whether both images have been received
    bool has_a_;
    bool has_b_;
    bool has_c_;
    bool has_d_;

    bool change;
    bool changedet;

    /// flag to remember, which image to re-publish
    bool publish_a_;

    /// Value minHessian for SurfDetector
    int hessianThreshold;

    template <typename T>
    void updateParam(const std::string& name, T value);
    template <typename T>
    void updateParamdet(const std::string& name, T value);

    struct State : public Memento {

        std::string key;
        std::string des;
        std::map<std::string, std::vector<vision::Parameter> > params_detector;
        std::map<std::string, std::vector<vision::Parameter> > params_extractor;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;


};

} // namespace csapex

#endif // OBJECT_DETECTION_H
