/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/view/utility/color.hpp>
using namespace csapex::connection_types;


namespace csapex
{

class PointIndicesToROIs : public Node
{
public:
    PointIndicesToROIs():
        dense_(false),
        use_projection_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_cloud_ = modifier.addInput<PointCloudMessage>("PointCloud");
        in_indices_ = modifier.addInput<GenericVectorMessage, pcl::PointIndices>("PointIndices");
        in_classification_ = modifier.addOptionalInput<GenericVectorMessage, int>("Classification");
        in_labels_ = modifier.addOptionalInput<GenericVectorMessage, std::string>("Labels");

        output_rois_ = modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareValue("fov/x", 60.0), fov_x_);
        params.addParameter(param::ParameterFactory::declareValue("fov/y",  60.0), fov_y_);
        params.addParameter(param::ParameterFactory::declareValue("centre/x", 0), c_x_);
        params.addParameter(param::ParameterFactory::declareValue("centre/y", 0), c_y_);
        params.addParameter(param::ParameterFactory::declareValue("width", 640), w_);
        params.addParameter(param::ParameterFactory::declareValue("height", 480), h_);

        params.addParameter(param::ParameterFactory::declareBool("flip rois", false), flip_rois_);
        params.addConditionalParameter(param::ParameterFactory::declareBool("use_projection",false),
                                       dense_, use_projection_);
    }

    void process()
    {
        PointCloudMessage::ConstPtr cloud(msg::getMessage<PointCloudMessage>(in_cloud_));


        clusters_ = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);



        if(msg::hasMessage(in_labels_)){
            labels_ = msg::getMessage<GenericVectorMessage, std::string>(in_labels_);
            apex_assert(labels_->size() == clusters_->size());
        }

        if(msg::hasMessage(in_classification_)){
            classification_ = msg::getMessage<GenericVectorMessage, int>(in_classification_);
            apex_assert(classification_->size() == clusters_->size());

            for(auto val : *classification_){
                if(colors_.find(val) == colors_.end()){
                    double r = 0,g = 0,b = 0;
                    color::fromCount(colors_.size(), r,g,b);
                    colors_.insert(std::make_pair(val, cv::Scalar(b,g,r)));
                }
            }
        }

        boost::apply_visitor (PointCloudMessage::Dispatch<PointIndicesToROIs>(this, cloud), cloud->value);

    }


    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<PointT>& cloud = *cloud_ptr;

        dense_ = cloud.width > 1 && cloud.height > 1;

        if(dense_) {
            inputDenseCloud<PointT>(cloud_ptr);
        } else  {
            inputSparseCloud<PointT>(cloud_ptr);
        }
    }

    template <class PointT>
    void inputDenseCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<PointT>& cloud = *cloud_ptr;
        std::shared_ptr<std::vector<RoiMessage>> out(new std::vector<RoiMessage>);


        std::size_t w = cloud.width;
        //        std::size_t h = cloud.height;

        int cluster_id = 0;
        std::vector<int>::const_iterator class_label;
        std::vector<std::string>::const_iterator label;

        if(classification_){
            class_label = classification_->begin();
        }

        if(labels_){
            label = labels_->begin();
        }

        for (auto cluster = clusters_->begin(); cluster != clusters_->end (); ++cluster, ++cluster_id) {
            auto indices = cluster->indices;

            std::vector<PointT const*> cluster_pts(indices.size());

            {
                double min_col = std::numeric_limits<double>::infinity();
                double max_col = -std::numeric_limits<double>::infinity();
                double min_row = std::numeric_limits<double>::infinity();
                double max_row = -std::numeric_limits<double>::infinity();
                double max_dist = -std::numeric_limits<double>::infinity();

                for(auto index = indices.begin(); index != indices.end(); ++index) {
                    int i = *index;
                    int row = i / w;
                    int col = i % w;
                    if(use_projection_){
                        const PointT& pt = cloud.points[*index];

                        /*  general:
                             *
                             *            / z
                             *           /
                             *          /
                             *         +-------
                             *         |      x
                             *         |
                             *         | y
                             *
                             *  for us:
                             *
                             *       z |  / x
                             *         | /
                             *         |/
                             *  -------+
                             *   y
                             *
                             */

                        //                double xx = pt.x;
                        //                double yy = pt.y;
                        //                double zz = pt.z;
                        double xx = -pt.y;
                        double yy = -pt.z;
                        double zz = pt.x;

                        col = c_x_ + fov_x_ * xx / zz;
                        row = c_y_ + fov_y_ * yy / zz;

                        if(pt.x > max_dist) max_dist = pt.x;
                    }
                    if(row < min_row) min_row = row;
                    if(row > max_row) max_row = row;
                    if(col < min_col) min_col = col;
                    if(col > max_col) max_col = col;
                }



                RoiMessage roi;

                cv::Rect rect;
                rect.x = min_col;
                rect.y = min_row;
                rect.width  = max_col - min_col;
                rect.height = max_row - min_row;

                if(flip_rois_) {
                    rect.y = h_ - rect.y - rect.height;
                }

                roi.value.setRect(rect);
                roi.value.setColor(cv::Scalar::all(0));
                if(classification_){
                    roi.value.setClassification(*class_label);
                    roi.value.setColor(colors_[*class_label]);
                    ++class_label;
                }
                else{
                    roi.value.setClassification(0);
                }
                if(labels_){
                    roi.value.setLabel(*label);
                    ++label;
                }
                out->push_back(roi);
            }

        }

        msg::publish<GenericVectorMessage, RoiMessage>(output_rois_, out);


        labels_.reset();
        classification_.reset();

    }

    template <class PointT>
    void inputSparseCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<PointT>& cloud = *cloud_ptr;
        std::shared_ptr<std::vector<RoiMessage>> out(new std::vector<RoiMessage>);

        int cluster_id = 0;
        std::vector<int>::const_iterator class_label;
        std::vector<std::string>::const_iterator label;
        if(classification_){
            class_label = classification_->begin();
        }

        if(labels_){
            label = labels_->begin();
        }
        for (auto cluster = clusters_->begin(); cluster != clusters_->end (); ++cluster, ++cluster_id) {
            auto indices = cluster->indices;
            double min_x = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            double max_dist = -std::numeric_limits<double>::infinity();

            for(auto index = indices.begin(); index != indices.end(); ++index) {
                const PointT& pt = cloud.points[*index];

                /*  general:
                 *
                 *            / z
                 *           /
                 *          /
                 *         +-------
                 *         |      x
                 *         |
                 *         | y
                 *
                 *  for us:
                 *
                 *       z |  / x
                 *         | /
                 *         |/
                 *  -------+
                 *   y
                 *
                 */

                //                double xx = pt.x;
                //                double yy = pt.y;
                //                double zz = pt.z;
                double xx = -pt.y;
                double yy = -pt.z;
                double zz = pt.x;

                double x = c_x_ + fov_x_ * xx / zz;
                double y = c_y_ + fov_y_ * yy / zz;

                if(y < min_y) min_y = y;
                if(y > max_y) max_y = y;
                if(x < min_x) min_x = x;
                if(x > max_x) max_x = x;

                if(pt.x > max_dist) max_dist = pt.x;
            }

            RoiMessage roi;
            cv::Rect rect;
            rect.x = min_x;
            rect.y = min_y;
            rect.width  = max_x - min_x;
            rect.height = max_y - min_y;

            if(flip_rois_) {
                rect.y = h_ - rect.y - rect.height;
            }

            roi.value.setRect(rect);
            roi.value.setColor(cv::Scalar(0, 0, max_dist / 10.0 * 255));
            if(classification_){
                roi.value.setClassification(*class_label);
                roi.value.setColor(colors_[*class_label]);
                ++class_label;
            }
            else{
                roi.value.setClassification(0);
            }
            if(labels_){
                roi.value.setLabel(*label);

                ++label;
            }
            out->push_back(roi);
        }

        msg::publish<GenericVectorMessage, RoiMessage>(output_rois_, out);

        labels_.reset();
        classification_.reset();
    }


private:
    Input* in_cloud_;
    Input* in_indices_;
    Input* in_labels_;
    Input* in_classification_;
    Output* output_rois_;

    double fov_x_;
    double fov_y_;
    int c_x_;
    int c_y_;
    int w_;
    int h_;

    bool flip_rois_;
    bool dense_;
    bool use_projection_;
    //    bool use_labels_;
    //    bool use_classification_;

    std::shared_ptr<std::vector<pcl::PointIndices> const>  clusters_;
    std::shared_ptr<std::vector<std::string> const> labels_;
    std::shared_ptr<std::vector<int> const> classification_;
    std::map<int, cv::Scalar> colors_;

};

}

CSAPEX_REGISTER_CLASS(csapex::PointIndicesToROIs, csapex::Node)

