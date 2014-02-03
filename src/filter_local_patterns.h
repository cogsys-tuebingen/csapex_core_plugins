#ifndef FILTER_LOCAL_PATTERNS_H
#define FILTER_LOCAL_PATTERNS_H

/// COMPONENTS
#include <csapex_vision/filter.h>
#include <utils_cv/local_patterns.hpp>

//class QDoubleSilder;
class QComboBox;
class QDoubleSlider;
class QLabel;

namespace vision_plugins {

class LocalPatterns : public csapex::Filter
{
    Q_OBJECT

public:
    LocalPatterns();

    virtual void filter(cv::Mat &img, cv::Mat &mask);
    virtual void insert(QBoxLayout *parent);

    void setState(csapex::Memento::Ptr memento);
    csapex::Memento::Ptr getState() const;

Q_SIGNALS:
    void timeUpdate(double value);

public Q_SLOTS:
    void update();
    void updateTime(double value);
private:
    utils_cv::LBP lbp_;
    utils_cv::LTP ltp_;

    QLabel        *time_;
    QDoubleSlider *slider_k_;
    QComboBox *combo_pattern_;

    class State : public csapex::Memento {
    public:
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Emitter &out) const;
    public:
        double k;
        int    index;
    };

    State state_;
    std::vector<cv::Scalar> colors_;


};

}

#endif // FILTER_LOCAL_PATTERNS_H
