/// HEADER
#include "difference_maximum.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::DifferenceMaximum, csapex::Node)

namespace {
    template<typename Tp>
    void neigh_diff(const cv::Mat &src, cv::Mat &dst, const bool abs = false)
    {
        dst = cv::Mat(src.rows, src.cols, src.type(), cv::Scalar::all(0.0));
        const static int N_X[] = {-1,-1,-1, 0, 0, 1, 1, 1};
        const static int N_Y[] = {-1, 0, 1,-1, 1,-1, 0, 1};

        for(int y = 0 ; y < src.rows ; ++y) {
            for(int x = 0 ; x < src.cols ; ++x) {
                Tp diff     = 0.0;
                Tp abs_diff = 0.0;
                Tp value    = src.at<Tp>(y,x);
                for(int n = 0 ; n < 8 ; ++n) {
                    int neighbour_x = x + N_X[n];
                    int neighbour_y = y + N_Y[n];
                    if(neighbour_x < 0 || neighbour_x >= src.cols ||
                            neighbour_y < 0 || neighbour_y >= src.rows)
                        continue;

                    Tp  neighbour_diff     = value - src.at<Tp>(neighbour_y, neighbour_x);
                    Tp  neighbour_abs_diff = std::abs(neighbour_diff);
                    if(neighbour_abs_diff > abs_diff) {
                        diff = neighbour_diff;
                        abs_diff = neighbour_abs_diff;

                    }
                    dst.at<Tp>(y,x) = abs ? abs_diff : diff;
                }
            }
        }
    }
}

DifferenceMaximum::DifferenceMaximum()
{
}

void DifferenceMaximum::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds));
    int  depth = in->value.type() & 7;
    bool abs_diff = readParameter<bool>("abs difference");

    switch (depth) {
    case CV_8U:
        neigh_diff<uchar>(in->value,out->value, true);
        break;
    case CV_16U:
        neigh_diff<ushort>(in->value,out->value, true);
        break;
    case CV_8S:
        neigh_diff<char>(in->value,out->value, abs_diff);
        break;
    case CV_16S:
        neigh_diff<short>(in->value,out->value, abs_diff);
        break;
    case CV_32S:
        neigh_diff<int>(in->value,out->value, abs_diff);
        break;
    case CV_32F:
        neigh_diff<float>(in->value,out->value, abs_diff);
        break;
    case CV_64F:
        neigh_diff<double>(in->value,out->value, abs_diff);
        break;
    default:
        break;
    }
    msg::publish(output_, out);
}

void DifferenceMaximum::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareBool("abs difference", false));
}
