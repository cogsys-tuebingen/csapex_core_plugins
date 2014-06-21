/// HEADER
#include "filter_static_mask.h"

/// PROJECT
#include <utils_param/parameter_factory.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::FilterStaticMask, csapex::Node)

using namespace csapex;

FilterStaticMask::FilterStaticMask()
    : state(this)
{
    addParameter(param::ParameterFactory::declareTrigger("create mask"), boost::bind(&FilterStaticMask::showPainter, this));
}

FilterStaticMask::~FilterStaticMask()
{
}

void FilterStaticMask::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "rows" << YAML::Value << mask_.rows;
    out << YAML::Key << "cols" << YAML::Value << mask_.cols;

    apex_assert_hard(mask_.type() == CV_8UC1);

    QByteArray raw = qCompress(mask_.data, mask_.rows * mask_.cols);
    out << YAML::Key << "rawdata" << YAML::Value << raw.toBase64().data();
 }

void FilterStaticMask::State::readYaml(const YAML::Node& node) {
    if(exists(node, "rawdata")){
        int rows, cols;
        node["rows"] >> rows;
        node["cols"] >> cols;

        std::string data;
        node["rawdata"] >> data;
        QByteArray raw = qUncompress(QByteArray::fromBase64(data.data()));

        cv::Mat(rows, cols, CV_8UC1, raw.data()).copyTo(mask_);

    } else  if(exists(node, "mask")) {
        std::string file;
        node["mask"] >> file;
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

Memento::Ptr FilterStaticMask::getChildState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void FilterStaticMask::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    apex_assert_hard(m.get());

    state = *m;

    triggerModelChanged();
}
