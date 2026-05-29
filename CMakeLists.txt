cmake_minimum_required(VERSION 3.28)
project(ModelDev)
set(CMAKE_CXX_STANDARD 17)
#设置尾缀
set(CMAKE_DEBUG_POSTFIX d)
#设定多平台相关
if (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
    #设定平台名称
    set(Current_Platform windows)
    #设定输出文件目录后缀
    set(CurrentOutPostfix windows)
    #设定真实输出文件目录后缀
    set(CurrentExternalPostfix ${CMAKE_BUILD_TYPE})
    #设定编码格式
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/execution-charset:GBK>")
else ()
    #设定平台名称
    set(Current_Platform linux)
    #设定输出文件目录后缀
    set(CurrentOutPostfix linux/${CMAKE_BUILD_TYPE})
    #设定真实输出文件目录后缀
    set(CurrentExternalPostfix "")
endif ()

#目录信息
set(OutputPath "_Build/out")

#设定可执行二进制文件的目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/${OutputPath}/${CurrentOutPostfix}/${CurrentExternalPostfix})
#设定存放编译出来的库文件的目录
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/${OutputPath}/${CurrentOutPostfix}/${CurrentExternalPostfix})
#添加自己的测试宏和日志输出判断 开启需要在Cmake配置中设置-DIS_LXN_TEST=1 -DIS_LOG_PRINT=1
if (NOT DEFINED IS_LXN_TEST)
    set(IS_LXN_TEST 0)
endif ()
add_definitions(-DIS_LXN_TEST=${IS_LXN_TEST})
if (NOT DEFINED IS_LOG_PRINT)
    set(IS_LOG_PRINT 0)
endif ()
add_definitions(-DIS_LOG_PRINT=${IS_LOG_PRINT})
#库文件目录
link_directories(${OutputPath}/${CurrentOutPostfix}/${CurrentExternalPostfix})
#包含文件
include_directories(
        "${PROJECT_SOURCE_DIR}"
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/include/Util"
        "${PROJECT_SOURCE_DIR}/include/ThirdParty"
        "${PROJECT_SOURCE_DIR}/include/ThirdParty/eigen3"
)

add_subdirectory(src)
add_subdirectory(Test)


