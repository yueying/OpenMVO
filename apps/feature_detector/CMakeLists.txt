INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # 确保在根目录的cmake中添加

PROJECT(feature_detector)

ADD_EXECUTABLE(feature_detector
               feature_detector.cpp)

INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/base/include)
INCLUDE_DIRECTORIES(${OPENMVO_INCLUDE_DIR}/libs/mvo/include)

SET(OPENMVO_LINKER_LIBS
	openmvo_base
	openmvo_mvo
	fast)

TARGET_LINK_LIBRARIES(feature_detector ${OPENMVO_LINKER_LIBS} ${OpenCV_LIBS})



# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(feature_detector PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(feature_detector  PROPERTIES PROJECT_LABEL "(app) feature_detector")
endif(ENABLE_SOLUTION_FOLDERS)



