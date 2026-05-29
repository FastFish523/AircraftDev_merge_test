add_executable(TGC_Test main.cpp)
target_link_libraries(TGC_Test PRIVATE TGC Utils)

add_executable(TGC_Dive_Test dive_main.cpp)
target_link_libraries(TGC_Dive_Test PRIVATE TGC Utils)

add_executable(TGC_Boost_Test Boosttest.cpp)
target_link_libraries(TGC_Boost_Test PRIVATE TGC Utils)

