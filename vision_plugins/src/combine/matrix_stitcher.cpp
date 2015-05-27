/// HEADER
#include "matrix_stitcher.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <boost/assign.hpp>

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::MatrixStitcher, csapex::Node)

namespace {
    void stitch(const cv::Mat &src1, const cv::Mat &src2,
                const bool vertical,
                const unsigned int offset,
                cv::Mat &dst)
    {
        apex_assert(src1.type() == src2.type());
        int rows = std::max(src1.rows, src2.rows);
        int cols = std::max(src1.cols, src2.cols);
        int pad_rows = 0;
        int pad_cols = 0;
        if(vertical) {
            pad_rows = offset;
            dst = cv::Mat(rows * 2 + offset,
                          cols,
                          src1.type(),
                          cv::Scalar::all(0));
        } else {
            pad_cols = offset;
            dst = cv::Mat(rows,
                          cols * 2 + offset,
                          src1.type(),
                          cv::Scalar::all(0));
        }

        /// CENTERING
        int x_off = (cols - src1.cols) / 2;
        int y_off = (rows - src1.rows) / 2;
        cv::Rect roi_rect(x_off, y_off, src1.cols, src1.rows);
        cv::Mat  roi(dst, roi_rect);
        src1.copyTo(roi);

        x_off = (cols - src2.cols) / 2 + pad_cols;
        y_off = (rows - src2.rows) / 2 + pad_rows;
        roi_rect = cv::Rect((vertical ? 0 : cols) + x_off,
                            (vertical ? rows : 0) + y_off,
                            src2.cols, src2.rows);
        roi = cv::Mat(dst, roi_rect);
        src2.copyTo(roi);

    }

}


MatrixStitcher::MatrixStitcher()
{
}

void MatrixStitcher::process()
{
    CvMatMessage::ConstPtr mat_1  = msg::getMessage<CvMatMessage>(matrix_1_);
    CvMatMessage::ConstPtr mat_2  = msg::getMessage<CvMatMessage>(matrix_2_);
    CvMatMessage::Ptr out(new CvMatMessage(mat_1->getEncoding(), mat_1->stamp_micro_seconds));

    if(!mat_1->getEncoding().matches(mat_2->getEncoding()))
        throw std::runtime_error("Matrices need the same encoding!");
    if(mat_1->value.type() != mat_2->value.type())
        throw std::runtime_error("Matrices need the same type!");

    Mode mode           = (Mode) readParameter<int>("mode");
    unsigned int offset = readParameter<int>("offset");
    switch(mode) {
    case HORIZONTAL:
        stitch(mat_1->value, mat_2->value, false, offset, out->value);
        break;
    case VERTICAL:
        stitch(mat_1->value, mat_2->value, true, offset, out->value);
        break;
    }

    msg::publish(stitched_, out);
}

void MatrixStitcher::setup(NodeModifier& node_modifier)
{
    matrix_1_ =  node_modifier.addInput<CvMatMessage>("matrix 1");
    matrix_2_ =  node_modifier.addInput<CvMatMessage>("matrix 2");
    stitched_ = node_modifier.addOutput<CvMatMessage>("stitched");
}

void MatrixStitcher::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> modes = boost::assign::map_list_of
            ("HORIZONTAL", HORIZONTAL)
            ("VERTICAL", VERTICAL);
    parameters.addParameter(param::ParameterFactory::declareParameterSet("mode", modes, (int) HORIZONTAL));
    parameters.addParameter(param::ParameterFactory::declareRange("offset", 0, 400, 0, 1));
}
