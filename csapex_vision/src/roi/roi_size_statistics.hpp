#pragma once
#include <csapex/model/node.h>

#include <csapex_opencv/roi.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

namespace csapex { namespace vision {

class ROISizeStatistics : public csapex::Node
{
    template<typename T>
    using Accumulator = boost::accumulators::accumulator_set<
            T,
            boost::accumulators::stats<
                    boost::accumulators::tag::min,
                    boost::accumulators::tag::max,
                    boost::accumulators::tag::mean,
                    boost::accumulators::tag::variance>>;

    template<typename T>
    static std::string formatAccumulator(const Accumulator<T>& accu)
    {
        std::ostringstream os;
        os << boost::accumulators::mean(accu);
        os << " (std.dev.: " << std::sqrt(boost::accumulators::variance(accu)) << ")";
        os << " (min: " << boost::accumulators::min(accu);
        os << ", max: " << boost::accumulators::max(accu) << ")";
        return os.str();
    }

    struct Statistics
    {
        void reset();
        void update(const Roi& roi);
        std::string format() const;

        std::size_t count;
        Accumulator<int> height;
        Accumulator<int> width;
        Accumulator<int> area;
        Accumulator<float> ratio;
    };

    enum class BinType { HEIGHT, WIDTH, AREA };

public:
    ROISizeStatistics();
    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    void resetStats();
    void saveStats();
    void updateStats(const Roi& roi);
    std::string formatStats() const;

private:
    Input* in_rois_;
    Output* out_info_;
    Output* out_histogram_;

    int max_width_;
    int max_height_;
    int histogram_bin_size_;
    std::string output_path_;
    int bin_count_;
    BinType bin_type_;

    Statistics global_stats_;
    cv::Mat global_histogram_;
    std::vector<Statistics> bin_stats_;
};

}}
