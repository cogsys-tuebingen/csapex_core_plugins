/// HEADER
#include "bar_code_reader.h"

/// COMPONENT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <zbar.h>

CSAPEX_REGISTER_CLASS(csapex::BarCodeReader, csapex::Node)

using namespace csapex;
using namespace connection_types;

BarCodeReader::BarCodeReader()
{}

void BarCodeReader::setupParameters(Parameterizable &params)
{
    params.addParameter(param::ParameterFactory::declareBool("republish",
                                                             param::ParameterDescription("publish the last detected code again, "
                                                                                         "if no new code has been detected "),
                                                             false));
}

void BarCodeReader::process()
{
    CvMatMessage::ConstPtr msg = msg::getMessage<CvMatMessage>(in_img);

    if(msg->value.channels() != 1) {
        throw std::runtime_error("Input must be 1-channel image!");
    }

    bool republish = readParameter<bool>("republish");
    bool published = false;

    zbar::ImageScanner scanner;
    // configure the reader
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // wrap image data
    int w = msg->value.cols;
    int h = msg->value.rows;
    zbar::Image image(w, h, "Y800", msg->value.data, w * h);

    // scan the image for barcodes
    scanner.scan(image);

    VectorMessage::Ptr out(VectorMessage::make<RoiMessage>());

    // extract results
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) {

        const zbar::Symbol& sym = *symbol;
        std::string data = sym.get_data();

        {
            int x = std::numeric_limits<int>::max();
            int y = std::numeric_limits<int>::max();
            int X = std::numeric_limits<int>::min();
            int Y = std::numeric_limits<int>::min();

            for(int i = 0; i < sym.get_location_size(); ++i) {
                int px = sym.get_location_x(i);
                int py = sym.get_location_y(i);
                x = std::min(x, px);
                X = std::max(X, px);
                y = std::min(y, py);
                Y = std::max(Y, py);
            }

            RoiMessage::Ptr msg(new RoiMessage);
            cv::Rect rect(x,y, X-x, Y-y);
            if(rect.x >= 0 && rect.y >= 0) {
                msg->value = Roi(rect, cv::Scalar(0,0,0));
                out->value.push_back(msg);
            }
        }

        //        if(data == data_) {
        //            if(lost) {
        //                lost = false;
        //            }

        //            if(!republish) {
        //                continue;
        //            }
        //        }

        msg::publish(out_str, data);
        published = true;

        data_ = data;
    }

    if(!published) {
        if(republish) {
            msg::publish(out_str, data_);
        } else {
            msg::publish(out_str, std::string(""));
        }
    }

    msg::publish(out_roi, out);

    // clean up
    image.set_data(nullptr, 0);

}


void BarCodeReader::setup(NodeModifier& node_modifier)
{
    in_img = node_modifier.addInput<CvMatMessage>("Image");

    out_str = node_modifier.addOutput<std::string>("String");
    out_roi = node_modifier.addOutput<VectorMessage, RoiMessage>("ROIs");
}
