#  主要为了确保当前CMakeLists.txt已经添加到了ROOT的CMakeLists.txt中，防止对源码树造成影响
#  Usage:  INCLUDE(AssureCMakeRootFile)
#

IF(NOT OPENMVO_SOURCE_DIR)
	MESSAGE(FATAL_ERROR "ERROR: Do not use this directory as 'source directory' in CMake, but the ROOT directory of the OPENMVO source tree.")
ENDIF(NOT OPENMVO_SOURCE_DIR)


