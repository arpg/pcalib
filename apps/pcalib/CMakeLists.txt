add_executable(pcalib-bin pcalib.cpp)
target_link_libraries(pcalib-bin pcalib)
set_target_properties(pcalib-bin PROPERTIES OUTPUT_NAME pcalib)