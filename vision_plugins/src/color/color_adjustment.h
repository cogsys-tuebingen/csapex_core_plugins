#ifndef FILTER_COLORCHANNEL_H
#define FILTER_COLORCHANNEL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex {
class ColorAdjustment : public csapex::Node
{
public:
    ColorAdjustment();

    void setup();
    void setupParameters();
    void process();

    void setState(Memento::Ptr memento);
    void setPreset();

private:
    void recompute();
    void update();

private:
    /// presets
    enum Preset{NONE, HSV, HSL, STD};

    /// connectors
    ConnectorIn   *input_;
    ConnectorOut  *output_;

    GenericStatePtr loaded_state_;

    Preset  active_preset_;

    Encoding current_encoding;
    std::vector<double> mins;
    std::vector<double> maxs;

    /// helpers
    void addLightness(cv::Mat &img);
};
}

#endif // FILTER_COLORCHANNEL_H
