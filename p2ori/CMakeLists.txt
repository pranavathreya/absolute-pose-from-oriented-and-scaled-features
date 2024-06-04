add_library(p2ori p2ori.h p2ori.cpp)
target_link_libraries(p2ori PRIVATE PoseLib::PoseLib)
