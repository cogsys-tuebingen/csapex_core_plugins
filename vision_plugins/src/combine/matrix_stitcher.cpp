/// HEADER
#include "matrix_stitcher.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
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
        apex_assert_hard(src1.type() == src2.type());
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
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    std::map<std::string, int> modes = boost::assign::map_list_of
            ("HORIZONTAL", HORIZONTAL)
            ("VERTICAL", VERTICAL);
    addParameter(param::ParameterFactory::declareParameterSet("mode", modes));
    addParameter(param::ParameterFactory::declareRange("offset", 0, 400, 0, 1));
}

void MatrixStitcher::process()
{
    CvMatMessage::Ptr mat_1  = matrix_1_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr mat_2  = matrix_2_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr out(new CvMatMessage(mat_1->getEncoding()));

    if(mat_1->getEncoding() != mat_2->getEncoding())
        throw std::runtime_error("Matrices need the same encoding!");
    if(mat_1->value.type() != mat_2->value.type())
        throw std::runtime_error("Matrices need the same type!");

    Mode mode           = (Mode) param<int>("mode");
    unsigned int offset = param<int>("offset");
    switch(mode) {
    case HORIZONTAL:
        stitch(mat_1->value, mat_2->value, false, offset, out->value);
        break;
    case VERTICAL:
        stitch(mat_1->value, mat_2->value, true, offset, out->value);
        break;
    }

    stitched_->publish(out);
}

void MatrixStitcher::setup()
{
    matrix_1_ =  modifier_->addInput<CvMatMessage>("matrix 1");
    matrix_2_ =  modifier_->addInput<CvMatMessage>("matrix 2");
    stitched_ = modifier_->addOutput<CvMatMessage>("stitched");
}
