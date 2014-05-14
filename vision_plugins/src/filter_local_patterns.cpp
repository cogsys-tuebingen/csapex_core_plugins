#include "filter_local_patterns.h"

/// COMPONENT
#include <csapex/utility/qt_helper.hpp>
#include <utils_cv/histogram.hpp>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QComboBox>
#include <QTime>
#include <boost/assign/list_of.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::LocalPatterns, csapex::Node)

using namespace vision_plugins;
using namespace csapex;

LocalPatterns::LocalPatterns()
{
    std::map<std::string, int> methods = boost::assign::map_list_of
            ("LBP", (int) LBP)
            ("LTP", (int) LTP);
    param::Parameter::Ptr method = param::ParameterFactory::declareParameterSet("method", methods);
    addParameter(method);


    boost::function<bool()> k_cond = (boost::bind(&param::Parameter::as<int>, method.get()) == LTP);
    addConditionalParameter(param::ParameterFactory::declareRange("k", 0.0, 1000.0, 5.0, 1.0), k_cond);

    colors_.push_back(utils_cv::histogram::COLOR_WHITE);
    colors_.push_back(utils_cv::histogram::COLOR_GREEN);
    colors_.push_back(utils_cv::histogram::COLOR_CYAN);
    colors_.push_back(utils_cv::histogram::COLOR_RED);

    Tag::createIfNotExists("Analysis");
    addTag(Tag::get("Analysis"));
}

void LocalPatterns::filter(Mat &img, Mat &mask)
{
    if(img.type() != CV_8UC1&&
       img.type() != CV_8UC2&&
       img.type() != CV_8UC3&&
       img.type() != CV_8UC4) {
        throw std::runtime_error("Image has to consist of chars!");
    }

    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    cv::Mat out;

    switch(param<int>("method")) {
    case LBP:
    {
        std::vector<cv::Mat> channel_hists;
        std::vector<int>     bins;

        Q_FOREACH(cv::Mat channel, channels) {
            lbp_.stdExtraction<uchar>(channel);

            channel_hists.push_back(lbp_.getHistogram());
            bins.push_back(256);
        }
        out = cv::Mat(600, 800, CV_8UC3, cv::Scalar::all(0));
        utils_cv::histogram::render_histogram<int>(channel_hists, bins, colors_, out);
    }
        break;

    case LTP:
    {
        std::vector<cv::Mat> channel_hists_pos;
        std::vector<cv::Mat> channel_hists_neg;
        std::vector<int>     bins;

        double k = param<double>("k");

        Q_FOREACH(cv::Mat channel, channels) {
            ltp_.stdExtraction<uchar>(channel, k);

            channel_hists_pos.push_back(ltp_.getPos());
            channel_hists_neg.push_back(ltp_.getNeg());
            bins.push_back(256);
        }

        out = cv::Mat(600, 1600, CV_8UC3, cv::Scalar::all(0));
        cv::Mat roi_pos(out, cv::Rect(0,0, 800, 600));
        cv::Mat roi_neg(out, cv::Rect(800,0, 800,600));
        utils_cv::histogram::render_histogram<int>
                (channel_hists_pos, bins, colors_, roi_pos);
        utils_cv::histogram::render_histogram<int>
                (channel_hists_neg, bins, colors_, roi_neg);
    }

        break;
    }

    img = out;
}


