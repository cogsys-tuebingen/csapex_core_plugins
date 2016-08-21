/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/param/interval_parameter.h>

using namespace csapex::connection_types;

namespace csapex
{

class Levels : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<CvMatMessage>("Image");
        out_ = modifier.addOutput<CvMatMessage>("Filtered Imag");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        addParameter(csapex::param::ParameterFactory::declareBool("auto levels", false),
                     [this](csapex::param::Parameter* p) {
            automatic_ = p->as<bool>();
        });
    }

    void setEncoding(const Encoding& e)
    {
        current_encoding_ = e;

        levels_.clear();
        removeTemporaryParameters();

        for(std::size_t ch = 0; ch < current_encoding_.channelCount(); ++ch) {
            auto channel = current_encoding_.getChannel(ch);
            csapex::param::Parameter::Ptr p;
            if(channel.fp) {
                throw std::logic_error("Levels for fp images is not yet implemented.");
                //                p = csapex::param::ParameterFactory::declareInterval<double>(channel.name, channel.min_f, channel.max_f,channel.min_f, channel.max_f, 0.01);
            } else {
                p = csapex::param::ParameterFactory::declareInterval(channel.name, channel.min_i, channel.max_i,channel.min_i, channel.max_i, 1);
            }
            addTemporaryParameter(p, [this](csapex::param::Parameter* p) {
                if(!automatic_) {
                    updateLut();
                }
            });
            levels_.push_back(std::dynamic_pointer_cast<param::IntervalParameter>(p));
        }
    }

    template <typename T>
    void map(T& v, std::size_t i, int lower, int upper) {
        if(i < (std::size_t) lower) {
            v = 0;
        } else if(i > (std::size_t) upper){
            v = 255;
        } else {
            v = 255.0 * (i - lower) / double(upper - lower);
        }
    }


    std::pair<int, int> getBounds(std::size_t channel)
    {
        return readParameter<std::pair<int,int>>(current_encoding_.getChannel(channel).name);
    }

    void updateLut()
    {
        std::size_t channels = levels_.size();
        if(channels != 1 && channels != 3) {
            throw std::logic_error("Levels is only implemented for 1 or 3 channels.");
        }

        int dim = 256;
        lut_ = cv::Mat(1, &dim, CV_8UC(channels));

        if(channels == 1) {
            std::pair<int, int> interval = getBounds(0);

            for (int i=0; i<dim; i++) {
                map(lut_.at<uchar>(i), i, interval.first, interval.second);
            }

        } else {
            for(std::size_t ch = 0; ch < channels; ++ch) {
                std::pair<int, int> interval = getBounds(ch);

                for (int i=0; i<dim; i++) {
                    map(lut_.at<cv::Vec3b>(i)[ch], i, interval.first, interval.second);
                }
            }
        }
    }

    void calculateLut1(const cv::Mat& img)
    {
        double min, max;
        cv::minMaxLoc(img, &min, &max);

        param::IntervalParameter::Ptr& p = levels_.at(0);
        std::pair<int, int> pair(min, max);
        p->set(pair);
    }

    void calculateLut3(const cv::Mat& img)
    {
        std::vector<std::pair<int, int>> pairs;
        pairs.resize(3);
        for(std::size_t ch = 0; ch < 3; ++ch) {
            pairs[ch].first = 255;
            pairs[ch].second = 0;
        }

        for(int row = 0; row < img.rows; ++row) {
            for(int col = 0; col < img.cols; ++col) {
                const auto& v = img_.at<cv::Vec3b>(row, col);
                for(std::size_t ch = 0; ch < 3; ++ch) {
                    pairs[ch].first = std::min(pairs[ch].first, (int) v[ch]);
                    pairs[ch].second = std::max(pairs[ch].second, (int) v[ch]);
                }
            }
        }

        for(std::size_t ch = 0; ch < 3; ++ch) {
            param::IntervalParameter::Ptr& p = levels_.at(ch);
            p->set(pairs[ch]);
        }
    }

    void calculateLut(const cv::Mat& img)
    {
        std::size_t channels = levels_.size();
        if(channels == 1) {
            calculateLut1(img);
        } else if(channels == 3){
            calculateLut3(img);
        } else {
            throw std::logic_error("Levels is only implemented for 1 or 3 channels.");
        }
        updateLut();
    }

    void process()
    {
        CvMatMessage::ConstPtr img_in = msg::getMessage<CvMatMessage>(in_);
        img_ = img_in->value;

        if(!img_in->getEncoding().matches(current_encoding_)) {
            setEncoding(img_in->getEncoding());
            if(!automatic_) {
                updateLut();
            }

        }
        if(automatic_){
            calculateLut(img_);
        }

        CvMatMessage::Ptr img_out(new CvMatMessage(img_in->getEncoding(), img_in->stamp_micro_seconds));

        cv::LUT(img_, lut_, img_out->value);

        msg::publish(out_, img_out);
    }

private:
    Input* in_;
    Output* out_;

    cv::Mat img_;

    bool automatic_;
    Encoding current_encoding_;
    std::vector<param::IntervalParameterPtr> levels_;
    cv::Mat lut_;
};

}

CSAPEX_REGISTER_CLASS(csapex::Levels, csapex::Node)

