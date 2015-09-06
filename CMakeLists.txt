# ----------------------------------------------------------------------------
#  根目录的CMake文件
#
#  fengbing <fengbing123@gmail.com>
#
#  注：CMake官网链接 http://www.cmake.org/
# ----------------------------------------------------------------------------

CMAKE_MINIMUM_REQUIRED (VERSION 2.8)#设定最小版CMake版本号

PROJECT (OpenMVO)#设定工程名称

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") # 添加本地CMake脚本目录

INCLUDE(cmake/script_version_number.cmake REQUIRED)#添加项目版本号

#设置路径
SET(OPENMVO_LIBS_ROOT "${CMAKE_SOURCE_DIR}/libs" CACHE INTERNAL "") 
SET(OPENMVO_SOURCE_DIR  ${CMAKE_CURRENT_SOURCE_DIR})
SET(OPENMVO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(OPENMVO_BINARY_DIR  ${PROJECT_BINARY_DIR})

#添加有用的宏
INCLUDE(cmake/UtilsMacros.cmake REQUIRED)# 添加一些有工具宏
INCLUDE(cmake/FilterFileLists.cmake REQUIRED)#用于文件筛选，便于后期进行跨平台
INCLUDE(cmake/DeclareOPENMVOLib.cmake REQUIRED)#用于自动生成项目

set(CMAKE_DEBUG_POSTFIX  "d")#设置Debug下lib的名称后添加d，如openmvo_XXX100d.lib

#用于项目中添加筛选器
IF (CMAKE_VERSION VERSION_GREATER 2.8.0)
	SET(ENABLE_SOLUTION_FOLDERS ON CACHE BOOL "Group projects under virtual folders of the compiler IDE (e.g. VisualStudio)")
ENDIF (CMAKE_VERSION VERSION_GREATER 2.8.0)

if (ENABLE_SOLUTION_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif (ENABLE_SOLUTION_FOLDERS)

#添加libs
FIND_PACKAGE(OpenCV REQUIRED) #通过CMake系统添加OpenCV
FIND_PACKAGE(g2o REQUIRED) #通过CMake系统添加g2o
FIND_PACKAGE(Boost REQUIRED)

include(cmake/script_eigen.cmake REQUIRED)  #添加Eigen
#include(cmake/script_sophus.cmake REQUIRED) #添加Sophus
include(cmake/script_fast.cmake REQUIRED)   #添加fast lib

#构建静态库还是动态库，目前先设置为静态库，后期更改为默认动态库
SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
IF(BUILD_SHARED_LIBS)
	SET(CMAKE_OPENMVO_BUILD_SHARED_LIB "#define OPENMVO_BUILT_AS_DLL")
	SET(CMAKE_OPENMVO_BUILD_SHARED_LIB_ONOFF 1)
ELSE(BUILD_SHARED_LIBS)
	SET(CMAKE_OPENMVO_BUILD_SHARED_LIB "/* #define OPENMVO_BUILT_AS_DLL */")
	SET(CMAKE_OPENMVO_BUILD_SHARED_LIB_ONOFF 0)
ENDIF(BUILD_SHARED_LIBS)

include(cmake/script_create_config_h.cmake REQUIRED)   # Build config.h
include(cmake/script_create_version_h.cmake REQUIRED)  # Build version.h

#设置include和link路径
INCLUDE_DIRECTORIES(".")#设置包含当前目录
INCLUDE_DIRECTORIES(${OPENMVO_CONFIG_FILE_INCLUDE_DIR})#包含全局配置文件config.h
LINK_DIRECTORIES("${CMAKE_BINARY_DIR}/lib")

#设置路径，保存lib和bin到同一目录
SET( LIBRARY_OUTPUT_PATH ${OPENMVO_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET( EXECUTABLE_OUTPUT_PATH ${OPENMVO_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )

# ----------------------------------------------------------------------------
#      					处理子目录
# ----------------------------------------------------------------------------
ADD_SUBDIRECTORY(3rdparty)   # 第三方库的路径
ADD_SUBDIRECTORY(libs)        # 本地C++ lib的路径

SET( BUILD_APPLICATIONS ON CACHE BOOL "If you only want the OPENMVO libraries, disable this.")
IF(BUILD_APPLICATIONS)
	ADD_SUBDIRECTORY(apps)    # 添加小应用程序目录
ENDIF(BUILD_APPLICATIONS)

ADD_SUBDIRECTORY(doc) #添加文档目录

#   添加单元测试
# ----------------------------------------------------------------------------
SET( BUILD_TESTING ON CACHE BOOL "Build OPENMVO tests")
IF( BUILD_TESTING)
	ADD_SUBDIRECTORY(tests)  # 添加单元测试
ENDIF( BUILD_TESTING)
