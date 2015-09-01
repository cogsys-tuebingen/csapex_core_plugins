/// HEADER
#include <csapex_vision/encoding.h>

/// SYSTEM
#include <sstream>
#include <boost/algorithm/string.hpp>

using namespace csapex;

Encoding::Encoding()
{}

std::string Encoding::toString() const
{
    std::stringstream s;

    std::string separator = "";
    s << "[";
    for(std::vector<Channel>::const_iterator it = channels_.begin(); it != channels_.end(); ++it) {
        const Channel& c = *it;
        s << separator << c.name;
        separator = ", ";
    }
    s << "]";

    return s.str();
}

Channel Encoding::getChannel(std::size_t index) const
{
    return channels_.at(index);
}

std::size_t Encoding::channelCount() const
{
    return channels_.size();
}

void Encoding::append(const Encoding &e)
{
    channels_.insert(channels_.end(), e.channels_.begin(), e.channels_.end());
}

void Encoding::push_back(const Channel &c)
{
    channels_.push_back(c);
}

bool Encoding::matches(const Encoding &rhs) const
{
    std::size_t n = channels_.size();
    if(n != rhs.channels_.size()) {
        return false;
    }

    for(unsigned i = 0; i < n; ++i) {
        if(getChannel(i) != rhs.getChannel(i)) {
            return false;
        }
    }

    return true;
}

namespace {
Channel makeChannel(const std::string& name)
{
    std::string key = name;
    boost::trim(key);

    for(std::vector<Channel>::const_iterator channel = enc::channel::g_channels.begin();
        channel != enc::channel::g_channels.end();
        ++channel) {
        const Channel& c = *channel;
        if(c.name == key) {
            return c;
        }
    }

    throw std::logic_error(std::string("channel '") + name + "' is unknown");
}
}

Encoding Encoding::fromString(const std::string& str)
{
    std::size_t n = str.length();

    assert(str[0] == '[');
    assert(str[n-1] == ']');
    std::string inner = str.substr(1, n-2);

    std::vector<std::string> tokens;
    boost::split(tokens, inner, boost::is_any_of(","));

    Encoding e;
    for(std::vector<std::string>::const_iterator token = tokens.begin(); token != tokens.end(); ++token) {
        e.push_back(makeChannel(*token));
    }

    return e;
}
