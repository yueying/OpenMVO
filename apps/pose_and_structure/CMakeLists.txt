INCLUDE(../../cmake/AssureCMakeRootFile.cmake)
PROJECT(pose_and_structure)

ADD_EXECUTABLE(pose_and_structure
               pose_and_structure.cpp)

INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/mvo/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/3rdparty/sophus/include)

SET(OPENMVO_LINKER_LIBS
	openmvo_base
	openmvo_mvo
	fast
	sophus)

TARGET_LINK_LIBRARIES(pose_and_structure ${OPENMVO_LINKER_LIBS} ${OpenCV_LIBS})


if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(pose_and_structure PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(pose_and_structure  PROPERTIES PROJECT_LABEL "(app) pose_and_structure")
endif(ENABLE_SOLUTION_FOLDERS)



