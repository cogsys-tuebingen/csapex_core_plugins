#pragma once

namespace csapex
{
namespace clustering
{
template <typename Data>
class NoOpValidator
{
public:
    template <typename... Args>
    NoOpValidator(Args&&...)
    {
    }

    constexpr bool start(const Data&)
    {
        return true;
    }

    constexpr bool extend(const Data&, const Data&)
    {
        return true;
    }

    constexpr bool finish()
    {
        return true;
    }
};

}  // namespace clustering
}  // namespace csapex
