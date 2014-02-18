/// HEADER
#include "filter_colorconvert.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/view/box.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QComboBox>
#include <QLabel>

CSAPEX_REGISTER_CLASS(vision_plugins::ColorConvert, csapex::Node)

using namespace vision_plugins;
using namespace csapex;
using namespace csapex::connection_types;

ColorConvert::ColorConvert()
    : combo_in_(NULL)
{
    cs_pair_to_operation_.insert(csiPair(csPair(RGB, BGR), (int) CV_RGB2BGR));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, RGB), (int) CV_BGR2RGB));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, YUV), (int) CV_RGB2YUV));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, YUV), (int) CV_BGR2YUV));
    cs_pair_to_operation_.insert(csiPair(csPair(YUV, RGB), (int) CV_YUV2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(YUV, BGR), (int) CV_YUV2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, HSV), (int) CV_RGB2HSV));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, HSV), (int) CV_BGR2HSV));
    cs_pair_to_operation_.insert(csiPair(csPair(HSV, RGB), (int) CV_HSV2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(HSV, BGR), (int) CV_HSV2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, HSL), (int) CV_RGB2HLS));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, HSL), (int) CV_BGR2HLS));
    cs_pair_to_operation_.insert(csiPair(csPair(HSL, RGB), (int) CV_HLS2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(HSL, BGR), (int) CV_HLS2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, MONO), (int) CV_RGB2GRAY));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, MONO), (int) CV_BGR2GRAY));
    cs_pair_to_operation_.insert(csiPair(csPair(MONO, RGB), (int) CV_GRAY2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(MONO, BGR), (int) CV_GRAY2BGR));

    cs_to_encoding_[YUV] = enc::yuv;
    cs_to_encoding_[RGB] = enc::rgb;
    cs_to_encoding_[BGR] = enc::bgr;
    cs_to_encoding_[HSL] = enc::hsl;
    cs_to_encoding_[HSV] = enc::hsv;
    cs_to_encoding_[MONO] = enc::mono;
}

ColorConvert::~ColorConvert()
{
}

void ColorConvert::fill(QBoxLayout *parent)
{
    setSynchronizedInputs(true);

    input_img_ = addInput<CvMatMessage>("Image");
    output_img_ = addOutput<CvMatMessage>("Image");

    combo_in_ = new QComboBox;
    combo_out_ = new QComboBox;
    fillCombo(combo_in_, index_to_cs_in_);
    fillCombo(combo_out_, index_to_cs_out_);

    parent->addWidget(new QLabel("Input CS"));
    parent->addWidget(combo_in_);
    parent->addWidget(new QLabel("Output CS"));
    parent->addWidget(combo_out_);
}

void ColorConvert::process()
{
    CvMatMessage::Ptr img = input_img_->getMessage<CvMatMessage>();

    /// MEMENTO
    state_.input_index = combo_in_->currentIndex();
    state_.output_index = combo_out_->currentIndex();

    csPair cspair;
    cspair.first  = index_to_cs_in_[state_.input_index];
    cspair.second = index_to_cs_out_[state_.output_index];

    CvMatMessage::Ptr out(new CvMatMessage(cs_to_encoding_[cspair.second]));

    if(img->getEncoding().size() != cs_to_encoding_[cspair.first].size()) {
        std::stringstream error;
        error << "Conversion not applicable! Input encoding #" << img->getEncoding().size() << ", target #"  <<  cs_to_encoding_[cspair.first].size();
        throw std::runtime_error(error.str());
    }

    if(cspair.first != cspair.second) {
        if(cs_pair_to_operation_.find(cspair) != cs_pair_to_operation_.end()) {
            int mode = cs_pair_to_operation_[cspair];
            cv::cvtColor(img->value, out->value, mode);

            if((int) out->getEncoding().size() != out->value.channels()) {
                throw std::runtime_error("Conversion didn't work!");
            }
        } else {
            throw std::runtime_error("Conversion not supported!");
        }
    } else {
        out = img;
    }


    output_img_->publish(out);
}

Memento::Ptr ColorConvert::getState() const
{
    return boost::shared_ptr<State>(new State(state_));
}

void ColorConvert::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;

    if(combo_in_) {
        combo_in_->setCurrentIndex(state_.input_index);
        combo_out_->setCurrentIndex(state_.output_index);
    }
}

bool ColorConvert::usesMask()
{
    return false;
}

void ColorConvert::fillCombo(QComboBox *combo, std::map<int, ColorSpace> &map)
{
    combo->addItem("BGR");
    map.insert(icsPair(combo->findText("BGR"), BGR));
    combo->addItem("RGB");
    map.insert(icsPair(combo->findText("RGB"), RGB));
    combo->addItem("HSV");
    map.insert(icsPair(combo->findText("HSV"), HSV));
    combo->addItem("HSL");
    map.insert(icsPair(combo->findText("HSL"), HSL));
    combo->addItem("YUV");
    map.insert(icsPair(combo->findText("YUV"), YUV));
    combo->addItem("MONO");
    map.insert(icsPair(combo->findText("MONO"), MONO));
}
