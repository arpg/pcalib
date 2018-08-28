add_executable(photocalib-bin photocalib.cpp)
target_link_libraries(photocalib-bin photocalib)
set_target_properties(photocalib-bin PROPERTIES OUTPUT_NAME photocalib)