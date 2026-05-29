file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(Utils SHARED ${DIR_SRCS})
target_compile_definitions(Utils PRIVATE SharedUtils_Build)