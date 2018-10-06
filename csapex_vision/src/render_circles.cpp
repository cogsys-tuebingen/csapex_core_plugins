
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/color.hpp>
#include <csapex_opencv/circle_message.h>
#include <csapex_opencv/cv_mat_message.h>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class RenderCircles : public Node
{
public:
    RenderCircles()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addMultiInput<CircleMessage, GenericVectorMessage>("Input");
        in_img_ = modifier.addInput<CvMatMessage>("image");
        out_ = modifier.addOutput<CvMatMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        //        params.addParameter(param::factory::declareRange("width",1,1920,640,1),
        //                            [this](param::Parameter* p){
        //            width_ = p->as<int>();
        //        });
        params.addParameter(param::factory::declareRange("line_thickness", 1, 10, 1, 1), thickness_);

        params.addParameter(param::factory::declareBool("different_colors", false), diff_colors_);

        params.addConditionalParameter(param::factory::declareColorParameter("color", 255, 0, 0), [this]() { return !diff_colors_; },
                                       [this](param::Parameter* p) {
                                           std::vector<int> c = p->as<std::vector<int>>();
                                           color_ = cv::Scalar(c[2], c[1], c[0]);
                                       });
    }

    void process() override
    {
        CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(in_img_);
        CvMatMessage::Ptr out(new CvMatMessage(*img));
        if (msg::isMessage<CircleMessage>(in_)) {
            CircleMessage::ConstPtr j_msg = msg::getMessage<CircleMessage>(in_);
            render(j_msg->value, out);

        } else {
            GenericVectorMessage::ConstPtr message = msg::getMessage<GenericVectorMessage>(in_);
            apex_assert(std::dynamic_pointer_cast<CircleMessage>(message->nestedType()));
            std::size_t n_circles = message->nestedValueCount();
            c_counter_ = 0;
            for (std::size_t i = 0; i < n_circles; ++i) {
                CircleMessage::ConstPtr circle = std::dynamic_pointer_cast<CircleMessage const>(message->nestedValue(i));
                render(circle->value, out);
            }
        }
        msg::publish(out_, out);
    }

    void render(const Circle& c, CvMatMessage::Ptr image)
    {
        cv::Point center((int)c.center_x, (int)c.center_y);
        cv::Scalar color = color_;
        if (diff_colors_) {
            double r = 0, g = 0, b = 0;
            ++c_counter_;
            color::fromCount(c_counter_, r, g, b);
            color = cv::Scalar(b, g, r);
        }
        cv::circle(image->value, center, (int)c.radius, color, thickness_, 8, 0);
    }

private:
    Input* in_;
    Input* in_img_;
    Output* out_;
    bool diff_colors_;
    //    std::size_t width_;
    //    std::size_t height_;
    int thickness_;
    std::size_t c_counter_;
    cv::Scalar color_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::RenderCircles, csapex::Node)
