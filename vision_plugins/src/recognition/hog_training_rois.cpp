/// HEADER
#include "hog_training_rois.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_core_plugins/vector_message.h>

/// SYSTEM
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::HOGTrainingRois, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


HOGTrainingRois::HOGTrainingRois()
{
}

void HOGTrainingRois::setupParameters()
{

    addParameter(param::ParameterFactory::declareRange("overlap",
                                                       param::ParameterDescription("Overlap in % of neg. training examples with \
                                                                                    pos. example in the middle."),
                                                       0, 100, 50, 1));
}

void HOGTrainingRois::setup()
{
    in_image_ = modifier_->addOptionalInput<CvMatMessage>("image");
    in_roi_   = modifier_->addInput<RoiMessage>("roi");
    out_      = modifier_->addOutput<GenericVectorMessage, RoiMessage>("rois");
}

void HOGTrainingRois::process()
{
    RoiMessage::ConstPtr in_roi = msg::getMessage<RoiMessage>(in_roi_);
    Roi roi = in_roi->value;
    std::shared_ptr< std::vector<RoiMessage> > out(new std::vector<RoiMessage>);

    int limit_x = std::numeric_limits<int>::max();
    int limit_y = std::numeric_limits<int>::max();

    if(msg::hasMessage(in_image_)) {
        CvMatMessage::ConstPtr in_image = msg::getMessage<CvMatMessage>(in_image_);
        limit_x = in_image->value.cols;
        limit_y = in_image->value.rows;
    }

    double overlap  = readParameter<int>("overlap") / 100.0;
    double dx       = (1.0 - overlap) * roi.w();
    double dy       = (1.0 - overlap) * roi.h();


    roi.setColor(cv::Scalar(0,255,0));
    roi.setClassification(0);
    RoiMessage msg;

    const static double xs[] = {0.0,0.0,1.0,-1.0};
    const static double ys[] = {1.0,-1.0,0.0,0.0};

    for(unsigned int i = 0 ; i < 4 ; ++i) {
        int x = roi.x() + xs[i] * dx;
        int y = roi.y() + ys[i] * dy;

        if(x >= 0 && y >= 0) {
            if(x + roi.w() <= limit_x &&
               y + roi.h() <= limit_y) {
                Roi r(x, y, roi.w(), roi.h(),cv::Scalar(0,0,255), 1);
                msg.value = r;
                out->push_back(msg);
            }
        }
    }

    msg.value = std::move(roi);
    out->push_back(msg);

    msg::publish<GenericVectorMessage, RoiMessage>(out_, out);
}

