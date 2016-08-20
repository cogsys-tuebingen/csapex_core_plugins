#include "filter_rois.hpp"

#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>

CSAPEX_REGISTER_CLASS(csapex::FilterROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

FilterROIs::FilterROIs()
{
}

void FilterROIs::setupParameters(Parameterizable& parameters)
{
    addParameter(param::ParameterFactory::declareInterval("width",
                                                          0, 640,
                                                          0, 0,
                                                          1),
                 width_);
    addParameter(param::ParameterFactory::declareInterval("height",
                                                          0, 480,
                                                          0, 0,
                                                          1),
                 height_);
    addParameter(param::ParameterFactory::declareInterval("aspect",
                                                          0.0, 10.0,
                                                          0.0, 0.0,
                                                          1.0),
                 aspect_);
}

void FilterROIs::setup(NodeModifier& node_modifier)
{
    in_rois_           = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
    out_rois_passed_   = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs (passed)");
    out_rois_rejected_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs (rejected)");
}

void FilterROIs::process()
{
    std::shared_ptr<std::vector<RoiMessage> const> rois_in = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
    std::shared_ptr<std::vector<RoiMessage>> rois_out_passed(new std::vector<RoiMessage>());
    std::shared_ptr<std::vector<RoiMessage>> rois_out_rejected(new std::vector<RoiMessage>());

    for (const RoiMessage& roi : *rois_in)
    {
        if (check(roi.value))
            rois_out_passed->emplace_back(roi);
        else
            rois_out_rejected->emplace_back(roi);
    }

    msg::publish<GenericVectorMessage, RoiMessage>(out_rois_passed_, rois_out_passed);
    msg::publish<GenericVectorMessage, RoiMessage>(out_rois_rejected_, rois_out_rejected);
}

bool FilterROIs::check(const Roi& roi) const
{
    bool passed = true;

    passed = passed & check_range(roi.w(), width_);
    passed = passed & check_range(roi.h(), height_);
    passed = passed & check_range(static_cast<double>(roi.w()) / roi.h(), aspect_);

    return passed;
}

