# ----------------------------------------------------------------------------
# 实例应用程序中的CMake文件，用于简单对app目录下的文件夹进行遍历，自定生成项目 
# ----------------------------------------------------------------------------
project(ALL_OPENMVO_APPS)

INCLUDE(../cmake/AssureCMakeRootFile.cmake) # 避免没有添加CMake文件到根目录下

# 遍历所有的子目录，然后进行处理
list_subdirectories(LIST_OPENMVO_APP_DIRS "${OPENMVO_SOURCE_DIR}/apps")

foreach(dir ${LIST_OPENMVO_APP_DIRS})
	SET(VAR_NAME "BUILD_APP_${dir}")
	SET(${VAR_NAME} ON CACHE BOOL "Build the app ${dir}")
	MARK_AS_ADVANCED(${VAR_NAME})

	if (${VAR_NAME})
	add_subdirectory(${dir})
	endif (${VAR_NAME})
endforeach(dir)
