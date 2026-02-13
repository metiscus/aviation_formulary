cmake_minimum_required(VERSION 3.14)
project(aviation_formulary VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Header-only library
add_library(aviation_formulary INTERFACE)
target_include_directories(aviation_formulary INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Enable testing
enable_testing()

# Add tests subdirectory
add_subdirectory(tests)

# Add examples subdirectory
add_subdirectory(examples)

# Installation
install(DIRECTORY include/ DESTINATION include)
install(TARGETS aviation_formulary EXPORT aviation_formularyTargets)
install(EXPORT aviation_formularyTargets
    FILE aviation_formularyTargets.cmake
    NAMESPACE aviation_formulary::
    DESTINATION lib/cmake/aviation_formulary
)
