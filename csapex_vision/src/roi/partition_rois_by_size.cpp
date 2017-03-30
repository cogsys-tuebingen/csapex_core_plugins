#include "partition_rois_by_size.hpp"

#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/model/node_modifier.h>
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
    param_width_ = param::ParameterFactory::declareRange(prefix + "/width", 0, 1920, 64, 1);
    param_height_ = param::ParameterFactory::declareRange(prefix + "/height", 0, 1200, 128, 1);

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

PartitionROIsBySize::PartitionROIsBySize() :
    num_scales_(0)
{}

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
    parameters.addParameter(param::ParameterFactory::declareRange("num_scales", 1, 64, 1, 1),
                            std::bind(&PartitionROIsBySize::updatedScales, this));

    static const std::map<std::string, int> available_methods = {
            { "none", static_cast<int>(Method::NONE) },
            { "any dimension", static_cast<int>(Method::ANY_DIMENSION) },
            { "all dimensions", static_cast<int>(Method::ALL_DIMENSIONS) },
            { "area", static_cast<int>(Method::AREA) },
            { "width", static_cast<int>(Method::WIDTH) },
            { "height", static_cast<int>(Method::HEIGHT) },
    };
    param::ParameterPtr method_parameter = param::ParameterFactory::declareParameterSet("method", available_methods,
                                                                                        static_cast<int>(Method::AREA));
    parameters.addParameter(method_parameter,
                            reinterpret_cast<int&>(method_));

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("default_scale", 1, 64, 1, 1),
                                       [this]() { return method_ == Method::NONE; },
                                       default_scale_);
}

void PartitionROIsBySize::process()
{
    std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    using ROIMessageList = std::shared_ptr<std::vector<RoiMessage>>;
    std::vector<ROIMessageList> out_rois;
    std::generate_n(std::back_inserter(out_rois), num_scales_,
                   []()
                   {
                       return std::make_shared<std::vector<RoiMessage>>();
                   });

    for (const RoiMessage& roi_msg : *in_rois)
    {
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

    using Size = std::pair<int, int>;
    std::vector<Size> sizes;
    std::transform(scales_.begin(), scales_.end(),
                   std::back_inserter(sizes),
                   [](const ScaleInfo& info) { return info.getSize(); });

    std::sort(sizes.begin(), sizes.end(),
              [this](const Size& a, const Size& b) -> bool
              {
                  switch (method_)
                  {
                      default:
                      case Method::AREA:
                      case Method::ANY_DIMENSION:
                      case Method::ALL_DIMENSIONS:
                          return a.first * a.second > b.first * b.second;
                      case Method::WIDTH:
                          return a.first > b.first;
                      case Method::HEIGHT:
                          return a.second > b.second;
                  }
              });

    for (std::size_t i = 1; i < num_scales_; ++i)
    {
        const auto& upper = sizes[i - 1];
        const auto& lower = sizes[i];

        double width = (upper.first + lower.second) / 2.0;
        double height = (upper.second + lower.second) / 2.0;

        switch (method_)
        {
            case Method::ANY_DIMENSION:
                if (roi.w() > width || roi.h() > height)
                    return i - 1;
                break;
            case Method::ALL_DIMENSIONS:
                if (roi.w() > width && roi.h() > height)
                    return i - 1;
                break;
            case Method::AREA:
                if (roi.w() * roi.h() > width * height)
                    return i - 1;
                break;
            case Method::WIDTH:
                if (roi.w() > width)
                    return i - 1;
                break;
            case Method::HEIGHT:
                if (roi.h() > height)
                    return i - 1;
                break;
            default:
                throw std::runtime_error("Unsupported method");
        }
    }

    return num_scales_ - 1;
}

void PartitionROIsBySize::updatedScales()
{
    const std::size_t new_scales = readParameter<int>("num_scales");

    if (new_scales > num_scales_)
    {
        for (std::size_t i = num_scales_; i < new_scales; ++i)
        {
            ScaleInfo scale;
            scale.create(this, *(this->node_modifier_), i);
            scales_.push_back(std::move(scale));
        }

    }
    else if (new_scales < num_scales_)
    {
        for (std::size_t i = new_scales; i < num_scales_; ++i)
        {
            auto itr = std::next(scales_.begin(), i);
            itr->destroy(this, *(this->node_modifier_));
            scales_.erase(std::next(scales_.begin(), i));
        }
    }

    num_scales_ = new_scales;
    getParameter<param::RangeParameter>("default_scale")->setMax<int>(new_scales);
}

