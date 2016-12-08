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
    bool                     valid;

    Entry() :
        cluster(-1),
        valid(false)
    {
    }

    // Note: intentially not virtual, called by templated class so we can avoid overhead
    inline Entry& merge(const Entry& other)
    {
        if (!valid)
        {
            index = other.index;
            valid = true;
        }
        indices.insert(indices.end(), other.indices.begin(), other.indices.end());
        return *this;
    }
};

struct EntryStatistical  : public Entry
{
    math::Mean<1>         depth_mean;
    math::Distribution<3> distribution;
    EntryStatistical() :
        Entry()
    {
    }

    inline EntryStatistical& merge(const EntryStatistical& other)
    {
        Entry::merge(other);
        depth_mean += other.depth_mean;
        distribution += other.distribution;
        return *this;
    }
};

struct EntryStatisticalColor : public EntryStatistical
{
    math::Mean<3> color_mean;
    EntryStatisticalColor() :
        EntryStatistical()
    {
    }

    inline EntryStatisticalColor& merge(const EntryStatisticalColor& other)
    {
        EntryStatistical::merge(other);
        color_mean += other.color_mean;
        return *this;
    }
};

struct EntryStatisticalMono : public EntryStatistical
{
    math::Mean<1> mono_mean;
    EntryStatisticalMono() :
        EntryStatistical()
    {
    }

    inline EntryStatisticalMono& merge(const EntryStatisticalMono& other)
    {
        EntryStatistical::merge(other);
        mono_mean += other.mono_mean;
        return *this;
    }
};




}

#endif // ENTRY_HPP
