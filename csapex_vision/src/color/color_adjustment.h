#ifndef FILTER_COLORCHANNEL_H
#define FILTER_COLORCHANNEL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_opencv/cv_mat_message.h>

namespace csapex
{
class ColorAdjustment : public csapex::Node
{
public:
    ColorAdjustment();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);
    virtual void process() override;

    void setParameterState(GenericStatePtr memento);
    void setPreset();

private:
    void recompute();
    void update();

private:
    /// presets
    enum Preset
    {
        NONE,
        HSV,
        HSL,
        STD
    };

    /// connectors
    Input* input_;
    Output* output_;

    GenericStatePtr loaded_state_;

    Preset active_preset_;

    Encoding current_encoding;
    std::vector<double> mins;
    std::vector<double> maxs;

    /// helpers
    void addLightness(cv::Mat& img);
};
}  // namespace csapex

#endif  // FILTER_COLORCHANNEL_H
