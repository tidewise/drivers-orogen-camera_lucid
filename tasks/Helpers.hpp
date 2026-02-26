#ifndef CAMERA_LUCID_HELPERS_HPP
#define CAMERA_LUCID_HELPERS_HPP

#include <camera_lucidTypes.hpp>

namespace camera_lucid {
    void validate(ImageConfig const& config);
    void validateIncompleteImageParameters(ImageConfig const& config);
}

#endif