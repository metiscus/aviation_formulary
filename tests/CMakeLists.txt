cmake_minimum_required(VERSION 3.14)

# Download and configure Google Test
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable CTest
include(CTest)

# Test executables
add_executable(aviation_tests
    test_aviation_formulary.cpp
)

add_executable(stability_tests
    test_numerical_stability.cpp
)

add_executable(new_module_tests
    test_new_modules.cpp
)

target_link_libraries(aviation_tests
    PRIVATE
        aviation_formulary
        GTest::gtest_main
)

target_link_libraries(stability_tests
    PRIVATE
        aviation_formulary
        GTest::gtest_main
)

target_link_libraries(new_module_tests
    PRIVATE
        aviation_formulary
        GTest::gtest_main
)

# Compiler warnings
if(MSVC)
    target_compile_options(aviation_tests PRIVATE /W4)
    target_compile_options(stability_tests PRIVATE /W4)
    target_compile_options(new_module_tests PRIVATE /W4)
else()
    target_compile_options(aviation_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(stability_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(new_module_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Discover tests
include(GoogleTest)
gtest_discover_tests(aviation_tests)
gtest_discover_tests(stability_tests)
gtest_discover_tests(new_module_tests)
