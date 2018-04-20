#ifndef ENCODING_H
#define ENCODING_H

/// PROJECT
#include <csapex_opencv/csapex_opencv_export.h>

/// SYSTEM
#include <string>
#include <vector>
#include <limits>

namespace csapex {

struct CSAPEX_OPENCV_EXPORT Channel {
    friend bool operator == (const Channel& lhs, const Channel& rhs) {
        return lhs.name == rhs.name && lhs.fp == rhs.fp;
    }

    friend bool operator != (const Channel& lhs, const Channel& rhs) {
        return ! operator == (lhs, rhs);
    }

    Channel(const std::string& name, int min, int max,
            std::vector<Channel>& container)
        : name(name), fp(false), min_i(min), max_i(max)
    {
        container.push_back(*this);
    }

    Channel(const std::string& name, float min, float max,
            std::vector<Channel>& container)
        : name(name), fp(true), min_f(min), max_f(max)
    {
        container.push_back(*this);
    }

    Channel(const std::string& name, double min, double max,
            std::vector<Channel>& container)
        : name(name), fp(true), min_f(min), max_f(max)
    {
        container.push_back(*this);
    }

    Channel(const Channel& copy)
        : name(copy.name), fp(copy.fp)
    {
        if(fp) {
            min_f = copy.min_f;
            max_f = copy.max_f;
        } else {
            min_i = copy.min_i;
            max_i = copy.max_i;
        }
    }

    Channel& operator = (const Channel& rhs)
    {
        name = rhs.name;
        fp = rhs.fp;
        if(fp) {
            min_f = rhs.min_f;
            max_f = rhs.max_f;
        } else {
            min_i = rhs.min_i;
            max_i = rhs.max_i;
        }
        return *this;
    }

    std::string name;
    bool fp;

    union {
        int min_i;
        float min_f;
    };
    union {
        int max_i;
        float max_f;
    };
};

class CSAPEX_OPENCV_EXPORT Encoding
{
public:
    template <typename Iterator>
    Encoding(Iterator begin, Iterator end)
        : channels_(begin, end)
    {}
    Encoding(std::initializer_list<Channel> channels)
        : channels_(channels)
    {}
    Encoding(const std::vector<Channel>& channels)
        : channels_(channels)
    {}

    Encoding();

    bool matches(const Encoding& encoding) const;

    Channel getChannel(std::size_t index) const;
    std::size_t channelCount() const;

    void append(const Encoding& e);
    void push_back(const Channel& c);

    std::string toString() const;
    static Encoding fromString(const std::string& str);

private:
    std::vector<Channel> channels_;
};

namespace enc {
namespace channel {
static std::vector<Channel> g_channels;

static const Channel gray("gray",0,255,  g_channels);

static const Channel red("r",0,255,  g_channels);
static const Channel green("g",0,255,  g_channels);
static const Channel blue("b",0,255,  g_channels);

static const Channel hue("h",0,255,  g_channels);
static const Channel saturation("s",0,255,  g_channels);
static const Channel l("l",0,255,  g_channels);
static const Channel y("y",0,255,  g_channels);
static const Channel u("u",0,255,  g_channels);
static const Channel v("v",0,255,  g_channels);

static const Channel depth("d", 0.0f, 255.0f, g_channels);
static const Channel unknown("?",0,1,  g_channels);

static const Channel a("a",0,255, g_channels);
static const Channel b("b",0,255, g_channels);

static const Channel L("L",  0.f, 1.f, g_channels);
static const Channel C("C",  0.f, 1.f, g_channels);
static const Channel H("H",  0.f, 1.f, g_channels);

static const Channel Integer("int", std::numeric_limits<int>::lowest(), std::numeric_limits<int>::max(), g_channels);
static const Channel Float("float", std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max(), g_channels);
static const Channel Double("double", std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(), g_channels);
static const Channel intensity("I", 0.f, std::numeric_limits<float>::max(), g_channels);
}

static const Encoding mono = { channel::gray };
static const Encoding bgr = { channel::blue, channel::green, channel::red };
static const Encoding unknown = bgr;
static const Encoding rgb = { channel::red, channel::green, channel::blue };
static const Encoding hsv = { channel::hue, channel::saturation, channel::v };
static const Encoding hsl = { channel::hue, channel::saturation, channel::l };
static const Encoding yuv = { channel::y, channel::u, channel::v };
static const Encoding depth = { channel::depth };
static const Encoding lab = {channel::l, channel::a, channel::b };
static const Encoding luv = {channel::l, channel::u, channel::v };
static const Encoding LCh = {channel::L, channel::C, channel::H };
static const Encoding pointXYZ = {channel::Float, channel::Float, channel::Float};
static const Encoding pointXYZNormal = {channel::Float, channel::Float, channel::Float,channel::Float, channel::Float, channel::Float};
static const Encoding pointXYZI = {channel::Float, channel::Float, channel::Float, channel::intensity};
static const Encoding pointXYZRGB = {channel::Float, channel::Float, channel::Float, channel::red, channel::green, channel::blue};
static const Encoding disparity = {channel::Float};
static const Encoding depth_f = {channel::Float};
}
}

#endif // ENCODING_H
