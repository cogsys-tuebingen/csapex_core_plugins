/*----------------------------------------------------------------------------

  LSD - Line Segment Detector on digital images

  Copyright (c) 2007-2011 rafael grompone von gioi <grompone@gmail.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Affero General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Affero General Public License for more details.

  You should have received a copy of the GNU Affero General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ----------------------------------------------------------------------------*/

/// HEADER
#include "lsd.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::LineSegmentDetector, csapex::Node)

LineSegmentDetector::LineSegmentDetector()
{
}

void LineSegmentDetector::process()
{
    connection_types::CvMatMessage::ConstPtr a = msg::getMessage<connection_types::CvMatMessage>(input_);

    //    cv::Mat image_cv = a->value;

    //    X = image_cv.rows;
    //    Y = image.cv.cols;
    //    /* get memory */

    //    image = new double [X*Y];
    //    if( image == NULL ) throw std::runtime_error("unsupported norm type");

    //    if(Image.type() == CV_8UC3) {
    //          Reading_image_impl<cv::Vec1b, Norm>(Image, Distance, Direction,
    //          Interpolation_coeff);
    //      } else if (Image.type() == CV_32FC3) {
    //          Reading_image_impl<cv::Vec1f, Norm>(Image, Distance, Direction,
    //          Interpolation_coeff);
    //      } else if (Image.type() == CV_64FC3) {
    //          Reading_image_impl<cv::Vec1d, Norm>(Image, Distance, Direction,
    //          Interpolation_coeff);
    //      } else {
    //          throw std::runtime_error("unsupported image type");
    //      }

    ////  template <typename PixelType, template<typename> class Norm>
    ////  void color_edge_detection::RCMG_computation_impl(cv::Mat Image, cv::Mat
    /// Distance, cv::Mat Direction, cv::Mat Interpolation_coeff) /  {
    ///* read data */
    // for(int y=0;y<Y;y++)
    //  for(int x=0;x<X;x++)
    //    image[ x + y * X ] = (double) image_cv.at<image_cv.type()>(x,y);
}

void LineSegmentDetector::setup(csapex::NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::CvMatMessage>("Image");

    output_ = node_modifier.addOutput<connection_types::CvMatMessage>("Image");
}
