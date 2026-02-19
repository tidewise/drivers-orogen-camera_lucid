rock_gtest(
    test_suite
        suite.cpp
        test_helpers.cpp
    SOURCES
        ../tasks/Helpers.cpp
    HEADERS
        ../tasks/Helpers.hpp
    DEPS_PKGCONFIG
        base-types
)

target_link_libraries(test_suite ${CAMERA_LUCID_TASKLIB_DEPENDENT_LIBRARIES})