INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保在根目录的cmake中添加

PROJECT(reprojection)

ADD_EXECUTABLE(reprojection
               reprojection.cpp)

INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/mvo/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/3rdparty/sophus/include)

SET(OPENMVO_LINKER_LIBS
	openmvo_base
	openmvo_mvo
	fast
	sophus)

TARGET_LINK_LIBRARIES(reprojection ${OPENMVO_LINKER_LIBS} ${OpenCV_LIBS})



# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(reprojection PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(reprojection  PROPERTIES PROJECT_LABEL "(app) reprojection")
endif(ENABLE_SOLUTION_FOLDERS)



