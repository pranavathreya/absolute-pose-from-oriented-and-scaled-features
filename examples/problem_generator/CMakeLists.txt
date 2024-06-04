include_directories(..)
add_library(problem_generator
    random.h
    problem_generator.h problem_generator.cpp
)
target_link_libraries(problem_generator affine2sift)
