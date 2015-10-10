INCLUDE(../../cmake/AssureCMakeRootFile.cmake)
PROJECT(depth_filter_app)

ADD_EXECUTABLE(depth_filter_app
               depth_filter_app.cpp)

INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/mvo/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/3rdparty/sophus/include)

SET(OPENMVO_LINKER_LIBS
	openmvo_base
	openmvo_mvo
	fast
	sophus)

TARGET_LINK_LIBRARIES(depth_filter_app ${OPENMVO_LINKER_LIBS} ${OpenCV_LIBS})


if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(depth_filter_app PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(depth_filter_app  PROPERTIES PROJECT_LABEL "(app) depth_filter_app")
endif(ENABLE_SOLUTION_FOLDERS)



