#ifndef ENTRY_HPP
#define ENTRY_HPP

#include <distribution.hpp>
#include <mean.hpp>

namespace csapex {
using DataIndex = std::array<int, 3>;

struct Entry {
    int                      cluster;
    DataIndex                index;
    std::vector<int>         indices;

    Entry() :
        cluster(-1)
    {
    }
};

struct EntryStatistical  : public Entry
{
    math::Distribution<3> distribution;
    EntryStatistical() :
        Entry()
    {
    }
};

template<std::size_t Dim>
struct EntryStatisticalIC : public EntryStatistical
{
    math::Mean<Dim> color_mean;
    EntryStatisticalIC() :
        EntryStatistical()
    {
    }
};

}

#endif // ENTRY_HPP
