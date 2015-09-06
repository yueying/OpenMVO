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
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(fast PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(fast  PROPERTIES PROJECT_LABEL "(3rd party) fast")
endif(ENABLE_SOLUTION_FOLDERS)