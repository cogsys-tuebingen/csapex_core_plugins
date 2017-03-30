/// HEADER
#include "covariance.h"

/// PROJECT
#include <cslibs_laser_processing/data/segment.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <cslibs_laser_processing/common/yaml-io.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::ScanCovariance, csapex::Node)


ScanCovariance::ScanCovariance()
{
}

void ScanCovariance::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("width", 100, 2000, 1000, 1));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("height", 100, 2000, 1000, 1));
}

void ScanCovariance::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<GenericVectorMessage, Segment>("Segments");
    output_ = node_modifier.addOutput<CvMatMessage>("Debug Output");
}

void ScanCovariance::process()
{
    std::shared_ptr<std::vector<Segment> const> segment_msg = msg::getMessage<GenericVectorMessage, Segment>(input_);
    const std::vector<Segment>& segments = *segment_msg;

    CvMatMessage::Ptr output(new CvMatMessage(enc::bgr, "unknown", 0));
    int w = readParameter<int>("width");
    int h = readParameter<int>("height");

    int n = segments.size();
    int max_comp = 2;

    cv::Mat pcaset(n, 2, CV_32F, cv::Scalar::all(0));

    for(int i = 0; i < n; ++i) {
        const Segment& segment = segments[i];

        const Eigen::Vector2d from(segment.rays.front().posX(), segment.rays.front().posY());
        const Eigen::Vector2d to(segment.rays.back().posX(), segment.rays.back().posY());

        Eigen::Vector2d delta = to - from;

        pcaset.at<float>(i, 0) = delta(0);
        pcaset.at<float>(i, 1) = delta(1);
    }
    cv::PCA pca(pcaset, cv::Mat(), CV_PCA_DATA_AS_ROW, max_comp);

    cv::Point2f e1, e2;
    e1.x = pca.eigenvectors.at<float>(0, 0) * pca.eigenvalues.at<float>(0);
    e1.y = pca.eigenvectors.at<float>(0, 1) * pca.eigenvalues.at<float>(0);
    e2.x = pca.eigenvectors.at<float>(1, 0) * pca.eigenvalues.at<float>(1);
    e2.y = pca.eigenvectors.at<float>(1, 1) * pca.eigenvalues.at<float>(1);

    output->value = cv::Mat(h, w, CV_8UC1, cv::Scalar::all(255));

    cv::Point2f center(w/2., h/2.);

    cv::line(output->value, center, center + e1 * (w/4.), cv::Scalar(0,0,255), 6, CV_AA);
    cv::line(output->value, center, center + e2 * (w/4.), cv::Scalar(0,255,0), 6, CV_AA);

    msg::publish(output_, output);
}
