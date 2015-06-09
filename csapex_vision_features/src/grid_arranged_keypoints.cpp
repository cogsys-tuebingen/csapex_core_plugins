/// COMPONENT
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <utils_param/range_parameter.h>
#include <utils_param/parameter_factory.h>
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;

class GridArrangedKeypoints : public csapex::Node
{
public:
    GridArrangedKeypoints()
    {

    }

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_img = node_modifier.addInput<CvMatMessage>("Image");
        out_key = node_modifier.addOutput<csapex::connection_types::KeypointMessage>("Keypoints");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("dimension x", 1, 1000, 64, 1));
        parameters.addParameter(param::ParameterFactory::declareRange("dimension y", 1, 1000, 48, 1));
        parameters.addParameter(param::ParameterFactory::declareRange("size", 0.0, 120.0, 0.0, 0.1));

        param::Parameter::Ptr advanced = param::ParameterFactory::declareBool("advanced", false);
        parameters.addParameter(advanced);

        std::function<bool()> cond_adv = [advanced]() { return advanced->as<bool>(); };

        parameters.addConditionalParameter(param::ParameterFactory::declareRange("class id", -1, 255, -1, 1),
                                           cond_adv);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("octave", 0, 12, 0, 1),
                                           cond_adv);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("response", 0, 12, 0, 1),
                                           cond_adv);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("angle", -1.0, 2 * M_PI, -1.0, 0.1),
                                           cond_adv);
    }

    virtual void process() override
    {
        CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(in_img);
        KeypointMessage::Ptr   out(new KeypointMessage);

        int dim_x = readParameter<int>("dimension x");
        int dim_y = readParameter<int>("dimension y");
        float cell_height   = in->value.rows / (float) dim_y;
        float cell_width    = in->value.cols / (float) dim_x;
        float cell_height_2 = cell_height * 0.5f;
        float cell_width_2  = cell_width * 0.5f;

        int   octave = 0;
        int   response = 0;
        int   class_id = -1;
        float angle = -1.f;
        float kp_size = std::min(cell_height, cell_width);


        if(readParameter<bool>("advanced")) {
            class_id = readParameter<int>("class id");
            octave = readParameter<int>("octave");
            response = readParameter<int>("response");
            angle = readParameter<double>("angle");
            if(angle < 0.f) {
                angle = -1.f;
            }
        }
        double size = readParameter<double>("size");
        if(size > 0.0) {
            kp_size = size;
        }

        out->value.resize(dim_x * dim_y, cv::KeyPoint());
        for(int i = 0 ; i < dim_y ; ++i) {
            for(int j = 0 ; j < dim_x ; ++j) {
                cv::KeyPoint &kp = out->value.at(i * dim_x + j);
                kp.pt.x = cell_width  * j + cell_width_2;
                kp.pt.y = cell_height * i + cell_height_2;
                kp.octave = octave;
                kp.response = response;
                kp.class_id = class_id;
                kp.angle = angle;
                kp.size = kp_size;
            }
        }
        msg::publish(out_key, out);
    }

private:
    Input*  in_img;
    Output* out_key;
};
}

CSAPEX_REGISTER_CLASS(csapex::GridArrangedKeypoints, csapex::Node)
