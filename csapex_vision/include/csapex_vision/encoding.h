#ifndef ENCODING_H
#define ENCODING_H

/// SYSTEM
#include <string>
#include <vector>
#include <boost/assign/list_of.hpp>

namespace csapex {

struct Channel {
    friend bool operator == (const Channel& lhs, const Channel& rhs) {
        return lhs.name == rhs.name;
    }

    friend bool operator != (const Channel& lhs, const Channel& rhs) {
        return ! operator == (lhs, rhs);
    }

    Channel(const std::string& name, int min, int max,
            std::vector<Channel>& container)
        : name(name), min(min), max(max)
    {
        container.push_back(*this);
    }

    std::string name;
    int min;
    int max;
};

class Encoding : public std::vector<Channel>
{
    typedef std::vector<Channel> Parent;

    friend bool operator != (const Encoding& lhs, const Encoding& rhs) {
        return ! operator == (lhs, rhs);
    }

    friend bool operator == (const Encoding& lhs, const Encoding& rhs) {
        if(lhs.size() != rhs.size()) {
            return false;
        }

        for(unsigned i = 0; i < lhs.size(); ++i) {
            if(lhs[i] != rhs[i]) {
                return false;
            }
        }

        return true;
    }

public:
    template <typename Iterator>
    Encoding(Iterator begin, Iterator end)
        : Parent(begin, end)
    {}

    Encoding();

    std::string toString() const;
    static Encoding fromString(const std::string& str);
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

static const Channel depth("d",0,255,  g_channels);
static const Channel unknown("?",0,1,  g_channels);
}

static const Encoding mono = boost::assign::list_of
        (channel::gray);

static const Encoding bgr = boost::assign::list_of
        (channel::blue)(channel::green)(channel::red);

static const Encoding unknown = bgr;

static const Encoding rgb = boost::assign::list_of
        (channel::red)(channel::green)(channel::blue);

static const Encoding hsv = boost::assign::list_of
        (channel::hue)(channel::saturation)(channel::v);

static const Encoding hsl = boost::assign::list_of
        (channel::hue)(channel::saturation)(channel::l);

static const Encoding yuv = boost::assign::list_of
        (channel::y)(channel::u)(channel::v);

static const Encoding depth = boost::assign::list_of
        (channel::depth);
}

}

#endif // ENCODING_H
