file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(TGC SHARED ${DIR_SRCS})
target_compile_definitions(TGC PRIVATE SharedTGC_Build)
target_link_libraries(TGC Utils)

