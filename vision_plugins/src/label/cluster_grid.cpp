/// HEADER
#include "cluster_grid.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_vision/utils/flood.h>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::ClusterGrid, csapex::Node)

ClusterGrid::ClusterGrid()
{
}

void ClusterGrid::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new CvMatMessage(enc::unknown, in->stamp_micro_seconds));

    out->value = cv::Mat(in->value.rows,
                         in->value.cols,
                         CV_32SC1,
                         cv::Scalar::all(0));

    int dim_x = readParameter<int>("dimension x");
    int dim_y = readParameter<int>("dimension y");
    int cell_height = in->value.rows / dim_y;
    int rest_height = in->value.rows % dim_y;
    int cell_width  = in->value.cols / dim_x;
    int rest_witdh  = in->value.cols % dim_x;

    int label = 0;
    cv::Rect roi;
    cv::Mat  roi_mat;
    for(int i = 0 ; i < dim_y ; ++i) {
        for(int j = 0 ; j < dim_x ; ++j) {
            roi = cv::Rect(cell_width  * j,
                           cell_height * i,
                           cell_width  + ((j == dim_x - 1) ? rest_witdh : 0),
                           cell_height + ((i == dim_y - 1) ? rest_height : 0));
            roi_mat = cv::Mat(out->value, roi);
            roi_mat.setTo(label);
            ++label;
        }
    }



    msg::publish(output_, out);
}

void ClusterGrid::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("image");
    output_ = node_modifier.addOutput<CvMatMessage>("cluster grid");
}

void ClusterGrid::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("dimension x", 1, 1000, 64, 1));
    parameters.addParameter(param::ParameterFactory::declareRange("dimension y", 1, 1000, 48, 1));
}


