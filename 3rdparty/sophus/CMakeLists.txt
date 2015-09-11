SET(PROJECT_NAME sophus)
PROJECT(${PROJECT_NAME})

file(GLOB sophus_source 
${PROJECT_SOURCE_DIR}/include/sophus/*.h 
${PROJECT_SOURCE_DIR}/src/*.cpp)

#  添加源码
SET(SOURCES  ${sophus_source})

# Add library
INCLUDE_DIRECTORIES(include/sophus)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${SOURCES})

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(sophus PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(sophus  PROPERTIES PROJECT_LABEL "(3rd party) sophus")
endif(ENABLE_SOLUTION_FOLDERS)