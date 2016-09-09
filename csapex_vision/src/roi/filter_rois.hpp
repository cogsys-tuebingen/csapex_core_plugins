#pragma once

#include <csapex/model/node.h>
#include <csapex_opencv/roi.h>
#include <iostream>

namespace csapex
{

class FilterROIs : public csapex::Node
{
public:
    FilterROIs();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    bool check(const Roi& roi) const;
    template<typename T>
    static bool check_range(const T& value, const std::pair<T, T>& range)
    {
        // check deactivated
        if (range.first == 0 && range.second == 0)
            return true;

        return value >= range.first && value <= range.second;
    }

private:
    Input* in_rois_;
    Output* out_rois_passed_;
    Output* out_rois_rejected_;

    std::pair<int, int> width_;
    std::pair<int, int> height_;
    std::pair<double, double> aspect_;
};

}
