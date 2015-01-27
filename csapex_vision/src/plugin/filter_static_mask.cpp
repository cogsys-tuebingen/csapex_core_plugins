/// HEADER
#include "filter_static_mask.h"

/// PROJECT
#include <utils_param/parameter_factory.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QByteArray>
#include <yaml-cpp/yaml.h>

CSAPEX_REGISTER_CLASS(csapex::FilterStaticMask, csapex::Node)

using namespace csapex;

FilterStaticMask::FilterStaticMask()
    : state(this)
{
    addParameter(param::ParameterFactory::declareTrigger("create mask"), std::bind(&FilterStaticMask::showPainter, this));
}

FilterStaticMask::~FilterStaticMask()
{
}

void FilterStaticMask::State::writeYaml(YAML::Node& out) const {
    out["rows"] = mask_.rows;
    out["cols"] = mask_.cols;

    apex_assert_hard(mask_.type() == CV_8UC1);

    QByteArray raw = qCompress(mask_.data, mask_.rows * mask_.cols);
    out["rawdata"] = raw.toBase64().data();
 }

void FilterStaticMask::State::readYaml(const YAML::Node& node) {
    if(node["rawdata"].IsDefined()){
        int rows = node["rows"].as<int>();
        int cols = node["cols"].as<int>();

        std::string data = node["rawdata"].as<std::string>();
        QByteArray raw = qUncompress(QByteArray::fromBase64(data.data()));

        cv::Mat(rows, cols, CV_8UC1, raw.data()).copyTo(mask_);

    } else if(node["mask"].IsDefined()) {
        std::string file = node["mask"].as<std::string>();
        mask_ = cv::imread(file, 0);
    }
}

void FilterStaticMask::showPainter()
{
    show_painter();
}

void FilterStaticMask::setMask(const cv::Mat &mask)
{
    mask.copyTo(state.mask_);
    setError(false);
}

cv::Mat FilterStaticMask::getMask() const
{
    return state.mask_.clone();
}

void FilterStaticMask::filter(cv::Mat& img, cv::Mat& mask)
{
    input(img);

    if(state.mask_.empty()) {
        setError(true, "No mask existing", EL_WARNING);
        return;
    }

    if(state.mask_.size != img.size) {
        setError(true, "The mask has not the same size as the image size", EL_WARNING);
        return;
    }

    if(!state.mask_.empty()) {
        if(mask.empty() || mask.size != state.mask_.size) {
            state.mask_.copyTo(mask);
        } else {
            mask = cv::min(mask, state.mask_);
        }
    }
}
