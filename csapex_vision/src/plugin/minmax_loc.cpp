/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
    class MinMaxLoc : public Node
    {
    public:
        MinMaxLoc()
        {

        }

        virtual void process() override
        {
            CvMatMessage::ConstPtr in  = msg::getMessage<connection_types::CvMatMessage>(in_);

            std::vector<cv::Mat> channels;
            cv::split(in->value, channels);

            std::shared_ptr<std::vector<double>> max(new std::vector<double>);
            std::shared_ptr<std::vector<double>> min(new std::vector<double>);
            std::shared_ptr<std::vector<int>> max_idx(new std::vector<int>);
            std::shared_ptr<std::vector<int>> min_idx(new std::vector<int>);

            for(cv::Mat &m : channels) {
                double      cmin,cmax;
                cv::Point   cmin_idx, cmax_idx;
                cv::minMaxLoc(m, &cmin, &cmax, &cmin_idx, &cmax_idx);
                max->push_back(cmax);
                min->push_back(cmin);
                max_idx->push_back(cmax_idx.x);
                max_idx->push_back(cmax_idx.y);
                min_idx->push_back(cmin_idx.x);
                min_idx->push_back(cmin_idx.y);
            }

            msg::publish<GenericVectorMessage, double>(out_max_, max);
            msg::publish<GenericVectorMessage, double>(out_min_, min);
            msg::publish<GenericVectorMessage, int>(out_max_idx_, max_idx);
            msg::publish<GenericVectorMessage, int>(out_min_idx_, min_idx);
        }

        virtual void setup(csapex::NodeModifier& node_modifier) override
        {
            in_           = node_modifier.addInput<CvMatMessage>("labels");
            out_max_      = node_modifier.addOutput<GenericVectorMessage, double>("max");
            out_min_      = node_modifier.addOutput<GenericVectorMessage, double>("min");
            out_max_idx_  = node_modifier.addOutput<GenericVectorMessage, int>("max_idx");
            out_min_idx_  = node_modifier.addOutput<GenericVectorMessage, int>("min_idx");
        }

        virtual void setupParameters(Parameterizable& parameters)
        {
        }

    private:
        Input  *in_;
        Output *out_max_;
        Output *out_min_;
        Output *out_max_idx_;
        Output *out_min_idx_;

        bool request_center_;
        void requestCenter();
    };

}
CSAPEX_REGISTER_CLASS(csapex::MinMaxLoc, csapex::Node)
