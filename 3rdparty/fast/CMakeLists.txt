SET(PROJECT_NAME fast)
PROJECT(${PROJECT_NAME})

file(GLOB fast_include ${PROJECT_SOURCE_DIR}/include/fast/*.h )

#  添加源码
SET(SOURCES  ${fast_include}
	         ${PROJECT_SOURCE_DIR}/src/fast_10.cpp
             ${PROJECT_SOURCE_DIR}/src/fast_10_score.cpp
             ${PROJECT_SOURCE_DIR}/src/nonmax_3x3.cpp
             ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)

#添加编译指令
ADD_DEFINITIONS(-D__SSE2__)#启用SSE2

# Add library
INCLUDE_DIRECTORIES(include)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${SOURCES})

# 设置将项目添加到包中
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rdparty")

