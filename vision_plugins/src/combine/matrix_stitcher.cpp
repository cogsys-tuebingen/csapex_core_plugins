/// HEADER
#include "matrix_stitcher.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <boost/assign.hpp>

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::MatrixStitcher, csapex::Node)

namespace {
    void stitch(const cv::Mat &src1, const cv::Mat &src2,
                const bool vertical, const unsigned int offset,
                cv::Mat &dst)
    {
        assert(src1.type() == src2.type());
        int rows = std::max(src1.rows, src2.rows) + (vertical ? offset : 0);
        int cols = std::max(src1.cols, src2.cols) + (vertical ? 0 : offset);
        dst = cv::Mat(rows * (vertical ? 2 : 1),
                      cols * (vertical ? 1 : 2),
                      src1.type(), cv::Scalar::all(0));

        cv::Rect roi_rect(0,0,src1.cols,src1.rows);
        cv::Mat  roi(dst, roi_rect);
        src1.copyTo(roi);

        roi_rect = vertical ? cv::Rect(0,rows + offset, src2.cols, src2.rows)
                            : cv::Rect(cols + offset,0, src2.cols, src2.rows);
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
    matrix_1_ =  addInput<CvMatMessage>("matrix 1");
    matrix_2_ =  addInput<CvMatMessage>("matrix 2");
    stitched_ = addOutput<CvMatMessage>("stitched");
}
