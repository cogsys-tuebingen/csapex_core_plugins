#pragma once

#include "../data/feature_color.hpp"
#include "../data/feature_helpers.hpp"
#include "color_differences.hpp"
#include "noop_validator.hpp"

namespace csapex
{
namespace clustering
{
enum ColorDifferenceType
{
    CIE76,
    CIE94Grahpics,
    CIE94Textiles
};

template <typename Data>
class ColorValidatorImpl
{
public:
    ColorValidatorImpl(ColorDifferenceType type, const std::array<double, 3>& weights, double threshold) : weights_(weights), threshold_(threshold)
    {
        switch (type) {
            default:
            case CIE76:
                difference_ = color_differences::CIE76;
                break;
            case CIE94Grahpics:
                difference_ = color_differences::CIE94Grahpics;
                break;
            case CIE94Textiles:
                difference_ = color_differences::CIE94Grahpics;
                break;
        }
    }

    bool start(const Data& data)
    {
        return true;
    }

    bool extend(const Data& center, const Data& data)
    {
        const auto& ref_feature = data.template getFeature<ColorFeature>();
        const auto& feature = data.template getFeature<ColorFeature>();

        double diff = ref_feature.difference(difference_, weights_, feature);
        return diff <= threshold_;
    }

    bool finish() const
    {
        return true;
    }

private:
    ColorFeature::DifferenceFunction difference_;
    std::array<double, 3> weights_;
    double threshold_;
};

template <typename Data>
struct ColorValidator : std::conditional<detail::tuple_contains<typename Data::FeatureList, ColorFeature>::value, ColorValidatorImpl<Data>, NoOpValidator<Data>>::type
{
    using BaseType = typename std::conditional<detail::tuple_contains<typename Data::FeatureList, ColorFeature>::value, ColorValidatorImpl<Data>, NoOpValidator<Data>>::type;

    using BaseType::BaseType;
};

}  // namespace clustering
}  // namespace csapex
