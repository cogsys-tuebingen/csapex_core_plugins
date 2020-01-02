#include "partition_rois_by_size.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex_opencv/roi_message.h>

CSAPEX_REGISTER_CLASS(csapex::vision::PartitionROIsBySize, csapex::Node)

using namespace csapex;
using namespace csapex::vision;
using namespace csapex::connection_types;

void PartitionROIsBySize::ScaleInfo::create(csapex::Node* node, csapex::NodeModifier& node_modifier, std::size_t index)
{
    const std::string prefix = std::string("scale_") + std::to_string(index + 1);

    index_ = index;
    output_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>(getNameForOutput(index));
    param_width_ = param::factory::declareRange(prefix + "/width", 0, 1920, 64, 1);
    param_height_ = param::factory::declareRange(prefix + "/height", 0, 1200, 128, 1);

    node->addTemporaryParameter(param_width_);
    node->addTemporaryParameter(param_height_);
}

void PartitionROIsBySize::ScaleInfo::destroy(csapex::Node* node, csapex::NodeModifier& node_modifier)
{
    node->removeTemporaryParameter(param_width_);
    node->removeTemporaryParameter(param_height_);

    if (msg::isConnected(output_))
        msg::disable(output_);
    else
        node_modifier.removeOutput(msg::getUUID(output_));
}

PartitionROIsBySize::PartitionROIsBySize() : num_scales_(0)
{
}

std::string PartitionROIsBySize::getNameForOutput(std::size_t i)
{
    std::ostringstream os;
    os << "ROIs Scale #" << (i + 1);
    return os.str();
}

void PartitionROIsBySize::setup(csapex::NodeModifier& node_modifier)
{
    in_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
}

void PartitionROIsBySize::setupParameters(csapex::Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareRange("num_scales", 1, 64, 1, 1), std::bind(&PartitionROIsBySize::updatedScales, this));

    static const std::map<std::string, int> available_methods = {
        { "none", static_cast<int>(Method::NONE) },
        { "area", static_cast<int>(Method::AREA) },
        { "width", static_cast<int>(Method::WIDTH) },
        { "height", static_cast<int>(Method::HEIGHT) },
    };
    param::ParameterPtr method_parameter = param::factory::declareParameterSet("method", available_methods, static_cast<int>(Method::AREA));
    parameters.addParameter(method_parameter, reinterpret_cast<int&>(method_));

    static const std::map<std::string, int> available_relative_to = {
        {
            "lower",
            static_cast<int>(RelativeTo::LOWER),
        },
        {
            "upper",
            static_cast<int>(RelativeTo::UPPER),
        },
        {
            "center",
            static_cast<int>(RelativeTo::CENTER),
        },
    };
    parameters.addConditionalParameter(param::factory::declareParameterSet("relative_to", available_relative_to, static_cast<int>(RelativeTo::LOWER)), [this]() { return method_ != Method::NONE; },
                                       reinterpret_cast<int&>(relative_to_));

    parameters.addConditionalParameter(param::factory::declareRange("default_scale", 1, 64, 1, 1), [this]() { return method_ == Method::NONE; }, default_scale_);
}

void PartitionROIsBySize::process()
{
    std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    using ROIMessageList = std::shared_ptr<std::vector<RoiMessage>>;
    std::vector<ROIMessageList> out_rois;
    std::generate_n(std::back_inserter(out_rois), num_scales_, []() { return std::make_shared<std::vector<RoiMessage>>(); });

    for (const RoiMessage& roi_msg : *in_rois) {
        const std::size_t scale = selectScale(roi_msg.value);
        out_rois[scale]->push_back(roi_msg);
    }

    for (std::size_t i = 0; i < num_scales_; ++i)
        msg::publish<GenericVectorMessage, RoiMessage>(scales_[i].getOutput(), out_rois[i]);
}

std::size_t PartitionROIsBySize::selectScale(const Roi& roi) const
{
    if (method_ == Method::NONE)
        return default_scale_ - 1;

    const auto get_value = [this](int width, int height) {
        switch (method_) {
            case Method::AREA:
                return width * height;
            case Method::WIDTH:
                return width;
            case Method::HEIGHT:
                return height;
            default:
                throw std::runtime_error("Unknown method");
        }
    };

    std::map<float, std::size_t> breaks;
    std::transform(scales_.begin(), scales_.end(), std::inserter(breaks, breaks.begin()), [&get_value](const ScaleInfo& info) {
        auto size = info.getSize();
        float value = get_value(size.first, size.second);

        return std::make_pair(value, info.getIndex());
    });

    float value = get_value(roi.w(), roi.h());

    using Iter = std::map<float, std::size_t>::const_iterator;
    Iter lower;
    Iter upper;
    std::tie(lower, upper) = breaks.equal_range(value);

    if (lower == breaks.begin())  // first element is >= -> select smallest scale
        return breaks.begin()->second;
    if (upper == breaks.end())  // no element is > -> select biggest scale
        return breaks.rbegin()->second;

    lower = std::prev(lower);  // first element <
    // upper                        // first element >

    switch (relative_to_) {
        case RelativeTo::LOWER:
            return lower->second;
        case RelativeTo::UPPER:
            return upper->second;
        case RelativeTo::CENTER: {
            const float center = (lower->first + lower->second) / 2.f;
            if (value >= center)
                return upper->second;
            else
                return lower->second;
        }
        default:
            throw std::runtime_error("Unknown relative_to");
    }
}

void PartitionROIsBySize::updatedScales()
{
    const std::size_t new_scales = readParameter<int>("num_scales");

    if (new_scales > num_scales_) {
        for (std::size_t i = num_scales_; i < new_scales; ++i) {
            ScaleInfo scale;
            scale.create(this, *(this->node_modifier_), i);
            scales_.push_back(std::move(scale));
        }

    } else if (new_scales < num_scales_) {
        for (std::size_t i = new_scales; i < num_scales_; ++i) {
            auto itr = std::next(scales_.begin(), i);
            itr->destroy(this, *(this->node_modifier_));
            scales_.erase(std::next(scales_.begin(), i));
        }
    }

    num_scales_ = new_scales;
    getParameter<param::RangeParameter>("default_scale")->setMax<int>(new_scales);
}
