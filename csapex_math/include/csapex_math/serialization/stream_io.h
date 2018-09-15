#ifndef PARAMETER_STREAM_IO_H
#define PARAMETER_STREAM_IO_H

/// COMPONENT
#include <csapex_math/model/vector.h>

/// SYSTEM
#include <ostream>

namespace csapex
{
namespace math
{
namespace linear
{

inline std::ostream& operator << (std::ostream& stream, const Vector& vector)
{
    for(std::size_t i = 0; i < vector.size(); ++i) {
        if(i > 0) {
            stream << ", ";
        }
        stream << vector[i];
    }
    return stream;
}

}
}
}


#endif // PARAMETER_STREAM_IO_H
