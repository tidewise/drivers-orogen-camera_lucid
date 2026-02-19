#include "../tasks/Helpers.hpp"

#include <gtest/gtest.h>

struct HelpersTest : public testing::Test {

    template <typename T>
    void assert_throws(std::function<void()> block, T const& exception)
    {
        bool exception_thrown{false};
        try {
            block();
        }
        catch (T const& e) {
            exception_thrown = true;
            ASSERT_STREQ(exception.what(), e.what());
        }

        ASSERT_EQ(true, exception_thrown);
    }
};

TEST_F(HelpersTest, it_throws_with_invalid_incomplete_image_config)
{
    camera_lucid::ImageConfig cfg;
    cfg.frame_timeout = base::Time::fromMicroseconds(1);
    cfg.max_incomplete_images = 10;
    cfg.check_image_status = base::Time::fromMicroseconds(11);

    assert_throws([&cfg]() { camera_lucid::validate(cfg); },
        std::runtime_error("check_image_status must be greater than the product "
                           "(max_incomplete_images + 1) * frame_timeout"));
}

TEST_F(HelpersTest, it_doesnt_throw_with_valid_incomplete_image_config)
{
    camera_lucid::ImageConfig cfg;
    cfg.frame_timeout = base::Time::fromMicroseconds(1);
    cfg.max_incomplete_images = 10;
    cfg.check_image_status = base::Time::fromMicroseconds(12);

    ASSERT_NO_THROW(camera_lucid::validate(cfg));
}
