#include "Helpers.hpp"

void camera_lucid::validate(ImageConfig const& config)
{
    validateIncompleteImageParameters(config);
}

void camera_lucid::validateIncompleteImageParameters(ImageConfig const& config)
{
    base::Time min_check_time = base::Time::fromMicroseconds(
        (config.max_incomplete_images + 1) * config.frame_timeout.toMicroseconds());
    if (config.check_image_status > min_check_time) {
        return;
    }

    throw std::runtime_error("check_image_status must be greater than the product "
                             "(max_incomplete_images + 1) * frame_timeout");
}
