/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <utils_vision/utils/rectangle_cluster.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>



namespace csapex {

using namespace connection_types;

class FilterROIs : public csapex::Node
{
public:
    FilterROIs();

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        output_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("filtered ROIs");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {

        parameters.addParameter(param::ParameterFactory::declareInterval("size_x",
                                param::ParameterDescription("Interval the size of the rois should be in."),
                                                             1, 1024, 0, 1024, 1),
                                size_x_);
        parameters.addParameter(param::ParameterFactory::declareInterval("size_y",
                                param::ParameterDescription("Interval the size of the rois should be in."),
                                                             1, 1024, 0, 1024, 1),
                                size_y_);
        parameters.addParameter(param::ParameterFactory::declareInterval("ratio",
                                param::ParameterDescription("Interval the ratio (x/y) of the rois should be in."),
                                                             0.0, 1.0, 0.0, 1.0, 0.01),
                                ratio_);

    }

    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> rois_in =
                msg::getMessage<GenericVectorMessage, RoiMessage>(input_);
        std::shared_ptr<std::vector<RoiMessage>> rois_out(new std::vector<RoiMessage>);
        for(const RoiMessage &roi : *rois_in) {
            const cv::Rect &r = roi.value.rect();
            bool size_x_fits = r.x >= size_x_.first &&
                               r.x < size_x_.second;
            bool size_y_fits = r.y >= size_y_.first &&
                               r.y < size_y_.second;

            if(!size_x_fits || !size_y_fits)
                continue;

            double ratio = r.x / r.y;
            bool ratio_fits = ratio >= ratio_.first &&
                              ratio < ratio_.second;

            if(!ratio_fits)
                continue;

            rois_out->emplace_back(roi);
        }
        msg::publish<GenericVectorMessage, RoiMessage>(output_, rois_out);
    }

private:
    Input* input_;
    Output* output_;

    std::pair<int, int> size_x_;
    std::pair<int, int> size_y_;
    std::pair<double, double> ratio_;
};
}

CSAPEX_REGISTER_CLASS(csapex::FilterROIs, csapex::Node)
