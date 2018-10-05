#include "roi_size_statistics.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <cslibs_vision/utils/color_functions.hpp>
#include <cslibs_vision/utils/heatmap.hpp>

#include <boost/algorithm/clamp.hpp>
#include <fstream>

CSAPEX_REGISTER_CLASS(csapex::vision::ROISizeStatistics, csapex::Node)

using namespace csapex;
using namespace csapex::vision;
using namespace csapex::connection_types;

void ROISizeStatistics::Statistics::reset()
{
    count = 0;
    height = Accumulator<int>();
    width = Accumulator<int>();
    area = Accumulator<int>();
    ratio = Accumulator<float>();
}

void ROISizeStatistics::Statistics::update(const Roi& roi)
{
    ++count;
    height(roi.h());
    width(roi.w());
    area(roi.h() * roi.w());
    ratio(float(roi.w()) / roi.h());
}

std::string ROISizeStatistics::Statistics::format() const
{
    std::ostringstream os;
    os << "Count : " << count << '\n';
    os << "Width : " << formatAccumulator(width) << "\n";
    os << "Height: " << formatAccumulator(height) << "\n";
    os << "Area  : " << formatAccumulator(area) << "\n";
    os << "Ratio : " << formatAccumulator(ratio) << "\n";
    return os.str();
}

ROISizeStatistics::ROISizeStatistics() : max_width_(640), max_height_(480), histogram_bin_size_(16), bin_count_(1)
{
}

void ROISizeStatistics::setup(csapex::NodeModifier& node_modifier)
{
    in_rois_ = node_modifier.addMultiInput<GenericVectorMessage, RoiMessage>("ROIs");
    out_info_ = node_modifier.addOutput<std::string>("Info");
    out_histogram_ = node_modifier.addOutput<CvMatMessage>("Histogram");
}

void ROISizeStatistics::setupParameters(csapex::Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareTrigger("reset"), std::bind(&ROISizeStatistics::resetStats, this));
    parameters.addParameter(param::factory::declareTrigger("save"), std::bind(&ROISizeStatistics::saveStats, this));

    parameters.addParameter(param::factory::declareFileOutputPath("path", "", "*.txt"), output_path_);

    parameters.addParameter(param::factory::declareRange("max_width", 1, 1920, 640, 1), [this](csapex::param::Parameter* param) {
        max_width_ = param->as<int>();
        resetStats();
    });
    parameters.addParameter(param::factory::declareRange("max_height", 1, 1080, 480, 1), [this](csapex::param::Parameter* param) {
        max_height_ = param->as<int>();
        resetStats();
    });
    parameters.addParameter(param::factory::declareRange("histogram_bin_size", 1, 128, 16, 1), [this](csapex::param::Parameter* param) {
        histogram_bin_size_ = param->as<int>();
        resetStats();
    });

    auto bin_count_param = param::factory::declareRange("bin/count", 0, 64, 0, 1).build();
    auto enable_bin_count = [bin_count_param]() { return bin_count_param->as<int>() > 0; };
    parameters.addParameter(bin_count_param, [this](csapex::param::Parameter* param) {
        bin_count_ = param->as<int>();
        resetStats();
    });

    static const std::map<std::string, int> available_bin_types = {
        { "height", static_cast<int>(BinType::HEIGHT) },
        { "width", static_cast<int>(BinType::WIDTH) },
        { "area", static_cast<int>(BinType::AREA) },
    };
    parameters.addConditionalParameter(param::factory::declareParameterSet("bin/type", available_bin_types, static_cast<int>(BinType::WIDTH)), enable_bin_count,
                                       [this](csapex::param::Parameter* param) {
                                           bin_type_ = static_cast<BinType>(param->as<int>());
                                           resetStats();
                                       });
}

void ROISizeStatistics::process()
{
    CvMatMessage::Ptr hist_msg;

    if (msg::isMessage<RoiMessage>(in_rois_)) {
        RoiMessage::ConstPtr in_roi = msg::getMessage<RoiMessage>(in_rois_);
        updateStats(in_roi->value);

        hist_msg = std::make_shared<CvMatMessage>(enc::unknown, in_roi->frame_id, in_roi->stamp_micro_seconds);
    } else {
        std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
        for (const RoiMessage& roi_msg : *in_rois)
            updateStats(roi_msg.value);

        if (!in_rois->empty())
            hist_msg = std::make_shared<CvMatMessage>(enc::unknown, in_rois->front().frame_id, in_rois->front().stamp_micro_seconds);
        else
            hist_msg = std::make_shared<CvMatMessage>(enc::unknown, "", 0);
    }

    hist_msg->value = global_histogram_;

    msg::publish<std::string>(out_info_, formatStats());
    msg::publish(out_histogram_, hist_msg);
}

void ROISizeStatistics::resetStats()
{
    global_stats_.reset();
    {
        const int width = max_width_ / histogram_bin_size_;
        const int height = max_height_ / histogram_bin_size_;
        global_histogram_ = cv::Mat::zeros(cv::Size(width, height), CV_32SC1);
    }

    bin_stats_.clear();
    bin_stats_.resize(bin_count_);
}

void ROISizeStatistics::updateStats(const Roi& roi)
{
    global_stats_.update(roi);
    {
        const int x = roi.w() / histogram_bin_size_;
        const int y = roi.h() / histogram_bin_size_;
        global_histogram_.at<int32_t>(cv::Point(x, y)) += 1;
    }

    std::size_t bin_index = 0;
    switch (bin_type_) {
        case BinType::HEIGHT:
            bin_index = std::size_t(boost::algorithm::clamp(roi.h(), 0, max_height_ - 1) / (max_height_ / float(bin_count_)));
            break;
        case BinType::WIDTH:
            bin_index = std::size_t(boost::algorithm::clamp(roi.w(), 0, max_width_ - 1) / (max_width_ / float(bin_count_)));
            break;
        case BinType::AREA:
            bin_index = std::size_t(boost::algorithm::clamp(roi.w() * roi.h(), 0, (max_width_ * max_height_) - 1) / ((max_width_ * max_height_) / float(bin_count_)));
            break;
        default:
            throw std::runtime_error("Unhandled BinType");
    }

    if (bin_count_ != 0)
        bin_stats_[bin_index].update(roi);
}

void ROISizeStatistics::saveStats()
{
    if (output_path_.empty())
        return;

    std::ofstream file(output_path_);
    file << formatStats();

    {
        const auto idx = output_path_.find_last_of(".");
        const std::string hist_path = output_path_.substr(0, idx) + ".hist.pgm";
        cv::imwrite(hist_path, global_histogram_);

        const std::string heatmap_path = output_path_.substr(0, idx) + ".heatmap.png";
        cv::Mat heatmap(global_histogram_.rows, global_histogram_.cols, CV_32FC3, cv::Scalar::all(0));
        cv::Mat float_histogram;
        global_histogram_.convertTo(float_histogram, CV_32FC1);
        cslibs_vision::heatmap::renderHeatmap(float_histogram, heatmap, &cslibs_vision::color::bezierColor<cv::Vec3f>);
        cv::resize(heatmap, heatmap, cv::Size(max_width_, max_height_));
        cv::imwrite(heatmap_path, heatmap);
    }
}

std::string ROISizeStatistics::formatStats() const
{
    std::ostringstream os;
    os << "ROI Statistics\n";
    os << "==============\n";
    os << "Total:\n";
    os << global_stats_.format();

    for (std::size_t i = 0; i < static_cast<std::size_t>(bin_count_); ++i) {
        os << "\n";
        os << "Bin #" << i << " break: ";
        switch (bin_type_) {
            case BinType::HEIGHT:
                os << i * (max_height_ / bin_count_) << " - " << (i + 1) * (max_height_ / bin_count_) << " (height)";
                break;
            case BinType::WIDTH:
                os << i * (max_width_ / bin_count_) << " - " << (i + 1) * (max_width_ / bin_count_) << " (width)";
                break;
            case BinType::AREA:
                os << i * ((max_width_ * max_height_) / bin_count_) << " - " << (i + 1) * ((max_width_ * max_height_) / bin_count_) << " (area)";
                break;
            default:
                throw std::runtime_error("Unhandled BinType");
        }
        os << "\n";
        os << bin_stats_[i].format();
    }

    return os.str();
}
