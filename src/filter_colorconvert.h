#ifndef FILTER_COLORCONVERT_H
#define FILTER_COLORCONVERT_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// COMPONENT
#include <csapex_vision/filter.h>
#include <csapex_vision/cv_mat_message.h>


class QComboBox;

namespace vision_plugins {
/**
 * @brief The ColorConvert class can be used to convert colored images from one color space
 *        to another.
 */
class ColorConvert : public csapex::BoxedObject
{
    Q_OBJECT

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
    virtual void fill(QBoxLayout *parent);
    /**
     * @brief See base class definition.
     */
    virtual void allConnectorsArrived();

    /// MEMENTO
    void                 setState(csapex::Memento::Ptr memento);
    csapex::Memento::Ptr getState() const;

protected:
    /// internal typdefs
    enum ColorSpace {YUV, RGB, BGR, HSL, HSV, MONO};
    typedef std::pair<ColorSpace, ColorSpace> csPair;
    typedef std::pair<csPair, int>     csiPair;
    typedef std::pair<int, ColorSpace> icsPair;

    std::map<csPair, int> cs_pair_to_operation_;
    std::map<ColorSpace, csapex::Encoding> cs_to_encoding_;
    std::map<int, ColorSpace> index_to_cs_in_;
    std::map<int, ColorSpace> index_to_cs_out_;

    QComboBox *combo_in_;
    QComboBox *combo_out_;

    csapex::ConnectorIn* input_img_;
    csapex::ConnectorOut* output_img_;

    virtual bool usesMask();
    void fillCombo(QComboBox *combo, std::map<int, ColorSpace> &map);

    /// MEMENTO
    class State : public csapex::Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["input_index"] >> input_index;
            node["output_index"] >> output_index;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "input_index" << YAML::Value << input_index;
            out << YAML::Key << "output_index" << YAML::Value << output_index;
        }

    public:
        int     input_index;
        int     output_index;
    };

    State state_;
};

}
#endif // FILTER_COLORCONVERT_H
