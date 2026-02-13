cmake_minimum_required(VERSION 3.14)

# Add example executable
add_executable(aviation_example
    example.cpp
)

target_link_libraries(aviation_example
    PRIVATE
        aviation_formulary
)

# Compiler settings
if(MSVC)
    target_compile_options(aviation_example PRIVATE /W4)
else()
    target_compile_options(aviation_example PRIVATE -Wall -Wextra -Wpedantic)
endif()
