#pragma once

#include <csapex/model/node.h>

#include <csapex_opencv/roi.h>

namespace csapex { namespace vision {

class PartitionROIsBySize : public csapex::Node
{
private:
    class ScaleInfo
    {
    public:
        void create(csapex::Node* node, csapex::NodeModifier& node_modifier, std::size_t index);
        void destroy(csapex::Node* node, csapex::NodeModifier& node_modifier);

        Output* getOutput() { return output_; }
        std::pair<int, int> getSize() const { return std::make_pair(param_width_->as<int>(), param_height_->as<int>()); };

    private:
        std::size_t index_ = 0;
        Output* output_ = nullptr;
        param::ParameterPtr param_width_;
        param::ParameterPtr param_height_;
    };

    enum class Method { NONE, ANY_DIMENSION, ALL_DIMENSIONS, AREA, WIDTH, HEIGHT };

public:

    PartitionROIsBySize();

    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    static std::string getNameForOutput(std::size_t i);
    void updatedScales();
    std::size_t selectScale(const Roi& roi) const;

private:
    Input* in_rois_;

    std::size_t num_scales_;
    std::vector<ScaleInfo> scales_;
    Method method_;
    int default_scale_;
};

}}
