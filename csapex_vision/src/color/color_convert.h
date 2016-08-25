#ifndef FILTER_COLORCONVERT_H
#define FILTER_COLORCONVERT_H

/// PROJECT
#include <csapex/msg/msg_fwd.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_opencv/cv_mat_message.h>


namespace csapex {
/**
 * @brief The ColorConvert class can be used to convert colored images from one color space
 *        to another.
 */
class ColorConvert : public csapex::Node
{
public:
    /**
     * @brief ColorConvert default constructor.
     */
    ColorConvert();
    /**
     * @brief ~ColorConvert destructor.
     */
    virtual ~ColorConvert();
    /**
     * @brief See base class definition.
     */
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    /**
     * @brief See base class definition.
     */
    virtual void process() override;

protected:
    /// internal typdefs
    enum ColorSpace {YUV, RGB, BGR, HSL, HSV, MONO, LAB, LCh};
    typedef std::pair<ColorSpace, ColorSpace> csPair;
    typedef std::pair<csPair, int>     csiPair;
    typedef std::pair<int, ColorSpace> icsPair;

    std::map<csPair, int> cs_pair_to_operation_;
    std::map<ColorSpace, csapex::Encoding> cs_to_encoding_;
    std::map<int, ColorSpace> index_to_cs_in_;
    std::map<int, ColorSpace> index_to_cs_out_;

    csapex::Input* input_img_;
    csapex::Output* output_img_;
};

}
#endif // FILTER_COLORCONVERT_H
