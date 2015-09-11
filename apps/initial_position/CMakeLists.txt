INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保在根目录的cmake中添加

PROJECT(initial_position)

ADD_EXECUTABLE(initial_position
               initial_position.cpp)

INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/mvo/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/3rdparty/sophus/include)

SET(OPENMVO_LINKER_LIBS
	openmvo_base
	openmvo_mvo
	fast
	sophus)

TARGET_LINK_LIBRARIES(initial_position ${OPENMVO_LINKER_LIBS} ${OpenCV_LIBS})



# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(initial_position PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(initial_position  PROPERTIES PROJECT_LABEL "(app) initial_position")
endif(ENABLE_SOLUTION_FOLDERS)



