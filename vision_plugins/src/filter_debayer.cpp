/// HEADER
#include "filter_debayer.h"

/// PROJECT
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QComboBox>
#include <QLabel>
#include <boost/assign/list_of.hpp>


CSAPEX_REGISTER_CLASS(vision_plugins::Debayer, csapex::Node)

using namespace vision_plugins;
using namespace csapex;

Debayer::Debayer()
{
    std::map<std::string, int> methods = boost::assign::map_list_of
            ("BayerBG2RGB", (int) CV_BayerBG2RGB)
            ("BayerGB2RGB", (int) CV_BayerGB2RGB)
            ("BayerRG2RGB", (int) CV_BayerRG2RGB)
            ("BayerGR2RGB", (int) CV_BayerGR2RGB)
            ("NNRG2RGB", 667);

    addParameter(param::ParameterFactory::declareParameterSet("method", methods));
}

QIcon Debayer::getIcon() const
{
    return QIcon(":/bayer.png");
}

void Debayer::filter(cv::Mat &img, cv::Mat &mask)
{
    int mode = param<int>("method");

    // assume 1 channel raw image comes in
    cv::Mat raw;
    cv::cvtColor(img, raw, CV_RGB2GRAY);

    if (mode == 667) {
        this->debayerAndResize(raw, img);
        cv::cvtColor(img, img, CV_BGR2RGB);
    }
    else {
        cv::cvtColor(raw, img, mode);
    }
}

bool Debayer::usesMask()
{
    return false;
}

// Debayer: bayer-Pattern
// - every pixel has it's own color filter (e.g. only sees red)
// - pixel returns brightness value
void Debayer::debayerAndResize(cv::Mat& source, cv::Mat& dest) {

    cv::MatIterator_<uchar> it = source.begin<uchar>(),
                         itEnd = source.end<uchar>();
    uchar* destination = (uchar*) dest.data;

    while(it != itEnd) {
        // r g r g r g
        // g b g b g b
        cv::MatIterator_<uchar> itLineEnd = it + 640;
        while(it != itLineEnd) {
            *destination = *it;
            ++it;
            ++destination;
            *destination = *it;
            ++it;
            ++destination;
            ++destination;
        }
        itLineEnd = it + 640;
        destination -= 320*3;
        while(it != itLineEnd) {
            // maybe add some green
            ++it;
            ++destination;
            ++destination;
            // add blue
            *destination = *it;
            ++it;
            ++destination;
        }
        destination += 320*3;
    }
}

