/// HEADER
#include "filter_static_mask.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_modifier.h>
#include <csapex/serialization/serialization.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QByteArray>
#include <yaml-cpp/yaml.h>

CSAPEX_REGISTER_CLASS(csapex::FilterStaticMask, csapex::Node)

using namespace csapex;

FilterStaticMask::FilterStaticMask()
{
}

void FilterStaticMask::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("create mask"), std::bind(&FilterStaticMask::showPainter, this));
}

void FilterStaticMask::showPainter()
{
    show_painter();
}

void FilterStaticMask::setMask(const cv::Mat &mask)
{
    mask.copyTo(mask_);
    node_modifier_->setNoError();
}

cv::Mat FilterStaticMask::getMask() const
{
    return mask_.clone();
}

void FilterStaticMask::filter(cv::Mat& img, cv::Mat& mask)
{
    input(img);

    if(mask_.empty()) {
        node_modifier_->setWarning("No mask existing");
        return;
    }

    if(mask_.size != img.size) {
        node_modifier_->setWarning("The mask has not the same size as the image size");
        return;
    }

    if(!mask_.empty()) {
        if(mask.empty() || mask.size != mask_.size) {
            mask_.copyTo(mask);
        } else {
            mask = cv::min(mask, mask_);
        }
    }
}

namespace csapex
{
class StaticMaskSerializer
{
public:
    static void serialize(const FilterStaticMask& mask, YAML::Node& doc)
    {
        doc["rows"] = mask.mask_.rows;
        doc["cols"] = mask.mask_.cols;

        apex_assert(mask.mask_.type() == CV_8UC1);

        QByteArray raw = qCompress(mask.mask_.data, mask.mask_.rows * mask.mask_.cols);
        doc["rawdata"] = raw.toBase64().data();
    }

    static void deserialize(FilterStaticMask& mask, const YAML::Node& doc)
    {
        if(doc["rawdata"].IsDefined()){
            int rows = doc["rows"].as<int>();
            int cols = doc["cols"].as<int>();

            std::string data = doc["rawdata"].as<std::string>();
            QByteArray raw = qUncompress(QByteArray::fromBase64(data.data()));

            cv::Mat(rows, cols, CV_8UC1, raw.data()).copyTo(mask.mask_);

        } else if(doc["mask"].IsDefined()) {
            std::string file = doc["mask"].as<std::string>();
            mask.mask_ = cv::imread(file, 0);
        }
    }
};
}

CSAPEX_REGISTER_SERIALIZER(csapex::FilterStaticMask, StaticMaskSerializer)
