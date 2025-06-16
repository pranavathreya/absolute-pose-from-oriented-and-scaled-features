file(
  GLOB_RECURSE
  numeric_files_header
  *.hpp
)
file(
  GLOB_RECURSE
  numeric_files_cpp
  *.cpp
)

add_library(openMVG_numeric
  ${numeric_files_header}
  ${numeric_files_cpp}
)