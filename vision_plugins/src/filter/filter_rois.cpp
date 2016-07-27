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

/// SYSTEM
#include <boost/assign.hpp>

namespace csapex {

using namespace connection_types;

class FilterROIs : public csapex::Node
{
public:
    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
        output_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("filtered ROIs");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {

        parameters.addParameter(param::ParameterFactory::declareInterval("size_x",
                                param::ParameterDescription("Interval the size of the rois should be in."),
                                                             1, 1024, 1, 1024, 1),
                                size_x_);
        parameters.addParameter(param::ParameterFactory::declareInterval("size_y",
                                param::ParameterDescription("Interval the size of the rois should be in."),
                                                             1, 1024, 1, 1024, 1),
                                size_y_);
        parameters.addParameter(param::ParameterFactory::declareInterval("ratio",
                                                             0.01, 1.0, 0.01, 1.0, 0.01),
                                ratio_);

        std::map<std::string, int> ratio_types =
                boost::assign::map_list_of("x/y",X_Y)("y/x",Y_X)("max",MAX);
        parameters.addParameter(param::ParameterFactory::declareParameterSet("ratio type",
                                                                              ratio_types,
                                                                             (int) X_Y),
                                ratio_type_);
    }

    virtual void process() override
    {
        std::shared_ptr<std::vector<RoiMessage> const> rois_in =
                msg::getMessage<GenericVectorMessage, RoiMessage>(input_);
        std::shared_ptr<std::vector<RoiMessage>> rois_out(new std::vector<RoiMessage>);
        for(const RoiMessage &roi : *rois_in) {
            const cv::Rect &r = roi.value.rect();
            bool size_x_fits = r.width >= size_x_.first &&
                               r.width < size_x_.second;
            bool size_y_fits = r.height >= size_y_.first &&
                               r.height < size_y_.second;

            if(!size_x_fits || !size_y_fits)
                continue;

            double ratio = 0.0;
            switch(ratio_type_) {
            case X_Y:
                ratio = r.width / (double) r.height;
                break;
            case Y_X:
                ratio = r.height / (double) r.width;
                break;
            case MAX:
                ratio = std::max(r.width / (double) r.height, r.height / (double) r.width );
                break;
            }

            bool ratio_fits = ratio >= ratio_.first &&
                              ratio < ratio_.second;

            if(!ratio_fits)
                continue;

            rois_out->push_back(roi);
        }
        msg::publish<GenericVectorMessage, RoiMessage>(output_, rois_out);
    }

private:
    Input* input_;
    Output* output_;

    enum RatioTypes {X_Y, Y_X, MAX};

    std::pair<int, int>       size_x_;
    std::pair<int, int>       size_y_;
    std::pair<double, double> ratio_;
    int                       ratio_type_;
};
}

CSAPEX_REGISTER_CLASS(csapex::FilterROIs, csapex::Node)
