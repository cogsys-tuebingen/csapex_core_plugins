/// HEADER
#include "plane_segmentation.h"

/// PROJECT
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/normals_message.h>
#include <csapex_opencv/cv_mat_message.h>

#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


/// SYSTEM
#include <opencv2/opencv.hpp>

CSAPEX_REGISTER_CLASS(csapex::PlaneSegmentation, csapex::Node)

using namespace csapex;
using namespace connection_types;

void PlaneSegmentation::setup(NodeModifier &node_modifier)
{

}

void PlaneSegmentation::setupParameters(Parameterizable &parameters)
{

}

void PlaneSegmentation::process()
{

}

template<class PointT>
void PlaneSegmentation::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

}

