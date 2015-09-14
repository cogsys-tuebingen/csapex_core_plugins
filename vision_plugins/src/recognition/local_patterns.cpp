/// HEADER
#include "local_patterns.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>

#include <csapex/model/connection_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>

#include <utils_vision/textures/lbp.hpp>
#include <utils_vision/textures/ltp.hpp>
#include <utils_vision/textures/wld.hpp>
#include <utils_vision/textures/homogenity.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::LocalPatterns, csapex::Node)

using namespace csapex;
using namespace vision_plugins;
using namespace connection_types;


LocalPatterns::LocalPatterns()
{

}

void LocalPatterns::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(in_);
    CvMatMessage::Ptr out(new CvMatMessage(enc::mono, in->stamp_micro_seconds));

    if(in->value.channels() > 1)
        throw std::runtime_error("Matrix must be one channel!");

    bool   b = readParameter<bool>("add border");
    double k = readParameter<double>("k");
    int    n = readParameter<int>("neighbours");
    int    r = readParameter<int>("radius");
    Type   t = (Type) readParameter<int>("descriptor");

    if(b) {
        cv::Mat working;
        int size = 1;
        switch(t) {
        case LBP:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LBP::standard(working, k, out->value);
            break;
        case LBP_EXT:
            size = r;
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LBP::extended(working, r, n, k, out->value);
            break;
        case LBP_VAR:
            size = r;
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LBP::var(working, r, n, out->value);
            break;
        case LBP_CS:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LBP::centerSymmetric(working, k, out->value);
            break;
        case LTP:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LTP::standard(working, k, out->value);
            out->setEncoding(enc::unknown);
            break;
        case LTP_EXT:
            size = r;
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LTP::extended(working, r, n , k , out->value);
            out->setEncoding(enc::unknown);
            break;
        case LTP_SHORT:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LTP::shortened(working, k, out->value);
            break;
        case LTP_CS:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::LTP::centerSymmetric(working, k, out->value);
            out->setEncoding(enc::unknown);
            break;
        case WLD:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::WLD::standard(working, out->value);
            break;
        case WLD_SHORT:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::WLD::shortened(working, out->value);
            break;
        case WLD_ORIENTED:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::WLD::oriented(working, out->value);
            break;
        case HOMOGENITY:
            cv::copyMakeBorder(in->value, working,
                               size, size, size, size,
                               cv::BORDER_REFLECT_101);
            utils_vision::Homogenity::standard(working, out->value);
            break;
        case HOMOGENITY_TEX:
            utils_vision::Homogenity::texture(in->value, out->value);
            break;
        default:
            throw std::runtime_error("Unknown local pattern!");
        }
    } else {
        switch(t) {
        case LBP:
            utils_vision::LBP::standard(in->value, k, out->value);
            break;
        case LBP_EXT:
            utils_vision::LBP::extended(in->value, r, n, k, out->value);
            break;
        case LBP_VAR:
            utils_vision::LBP::var(in->value, r, n, out->value);
            break;
        case LBP_CS:
            utils_vision::LBP::centerSymmetric(in->value, k, out->value);
            break;
        case LTP:
            utils_vision::LTP::standard(in->value, k, out->value);
            out->setEncoding(enc::unknown);
            break;
        case LTP_EXT:
            utils_vision::LTP::extended(in->value, r, n , k , out->value);
            out->setEncoding(enc::unknown);
            break;
        case LTP_SHORT:
            utils_vision::LTP::shortened(in->value, k, out->value);
            break;
        case LTP_CS:
            utils_vision::LTP::centerSymmetric(in->value, k, out->value);
            out->setEncoding(enc::unknown);
            break;
        case WLD:
            utils_vision::WLD::standard(in->value, out->value);
            break;
        case WLD_SHORT:
            utils_vision::WLD::shortened(in->value, out->value);
            break;
        case WLD_ORIENTED:
            utils_vision::WLD::oriented(in->value, out->value);
            break;
        case HOMOGENITY:
            utils_vision::Homogenity::standard(in->value, out->value);
            break;
        case HOMOGENITY_TEX:
            utils_vision::Homogenity::texture(in->value, out->value);
            break;
        default:
            throw std::runtime_error("Unknown local pattern!");
        }
    }
    msg::publish(out_, out);
}

void LocalPatterns::setup(NodeModifier &node_modifier)
{
    out_ = node_modifier.addOutput<CvMatMessage>("Local Patterns");
    in_  = node_modifier.addInput<CvMatMessage>("Mono");
}

void LocalPatterns::setupParameters(Parameterizable &parameters)
{
    std::map<std::string, int> types =
            boost::assign::map_list_of
            ("LBP", LBP)("LBP_EXT", LBP_EXT)("LBP_VAR", LBP_VAR)("LBP_CS", LBP_CS)
            ("LTP", LTP)("LTP_EXT", LTP_EXT)("LTP_SHORT", LTP_SHORT)("LTP_CS", LTP_CS)
            ("WLD", WLD)("WLD_SHORT", WLD_SHORT)("WLD_ORIENTED", WLD_ORIENTED)
            ("HOMOGENITY", HOMOGENITY)("HOMOGENITY_TEX", HOMOGENITY_TEX);

    param::Parameter::Ptr type =
            param::ParameterFactory::declareParameterSet("descriptor",
                                                         types,
                                                         (int) LBP);

    std::function<bool()> condition_threshold = [type]() {
        Type t = (Type) type->as<int>();
        return t != LBP_VAR &&
                t != WLD && t != WLD_ORIENTED && t != WLD_SHORT &&
                t != HOMOGENITY && t != HOMOGENITY_TEX;
    };

    std::function<bool()> condition_neighbours = [type]() {
        Type t = (Type) type->as<int>();
        return t == LBP_EXT || t == LBP_VAR ||
                t == LTP_EXT;
    };

    parameters.addParameter(type);

    parameters.addConditionalParameter(
                param::ParameterFactory::declareRange("k",
                                                      param::ParameterDescription("Center difference offset."),
                                                      -100.0, 100.0, 0.0, 0.1),
                condition_threshold);

    parameters.addConditionalParameter(
                param::ParameterFactory::declareRange("neighbours",
                                                      param::ParameterDescription("Amount of neighbours."),
                                                      2, 31, 8, 1),
                condition_neighbours);

    parameters.addConditionalParameter(
                param::ParameterFactory::declareRange("radius",
                                                      param::ParameterDescription("Radius of neighbourhood."),
                                                      1, 16, 8, 1),
                condition_neighbours);

    parameters.addParameter(
                param::ParameterFactory::declareBool("add border",
                                                     param::ParameterDescription("Add boarder to adjust pattern analysis size to image size."),
                                                     false));
}
