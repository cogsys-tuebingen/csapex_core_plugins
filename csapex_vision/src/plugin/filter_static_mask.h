#ifndef FILTER_STATIC_MASK_H
#define FILTER_STATIC_MASK_H

/// PROJECT
#include <csapex_vision/filter.h>

namespace csapex
{

class FilterStaticMask : public Filter
{
public:
    FilterStaticMask();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask) override;
    virtual void setupParameters(Parameterizable &parameters) override;

    void setMask(const cv::Mat& m);
    cv::Mat getMask() const;

private:
    void showPainter();

public:
    boost::signals2::signal<void(cv::Mat&)> input;
    boost::signals2::signal<void()> show_painter;

private:
    struct State : public Memento {
        cv::Mat mask_;

        State(FilterStaticMask* parent)
            : parent(parent)
        {}

        virtual void writeYaml(YAML::Node& out) const;
        virtual void readYaml(const YAML::Node& node);

        FilterStaticMask* parent;
    };

    State state;
};

} /// NAMESPACE

#endif // FILTER_STATIC_MASK_H
