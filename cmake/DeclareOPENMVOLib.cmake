# define_fblib_lib(): Declares an FBLIB library target:
#-----------------------------------------------------------------------
macro(define_fblib_lib name)
	internal_define_fblib_lib(${name} 0 0 ${ARGN}) # headers_only = 0, is_metalib=0
endmacro(define_fblib_lib)

# define_fblib_lib_header_only(): Declares an FBLIB headers-only library:
#-----------------------------------------------------------------------
macro(define_fblib_lib_header_only name)
	internal_define_fblib_lib(${name} 1 0 ${ARGN}) # headers_only = 1, is_metalib=0
endmacro(define_fblib_lib_header_only)

# define_fblib_metalib(): Declares an FBLIB meta-lib:
#-----------------------------------------------------------------------
macro(define_fblib_metalib name)
	internal_define_fblib_lib(${name} 1 1 ${ARGN}) # headers_only = 1, is_metalib=1
endmacro(define_fblib_metalib)

# Implementation of both define_fblib_lib() and define_fblib_lib_headers_only():
#-----------------------------------------------------------------------------
macro(internal_define_fblib_lib name headers_only is_metalib)
	INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

	# Allow programmers of fblib libs to change the default value of build_fblib_LIB, which is "ON" by default.
	SET(_DEFVAL "${DEFAULT_BUILD_fblib_${name}}")
	IF ("${_DEFVAL}" STREQUAL "")
		SET(_DEFVAL "ON")
	ENDIF ("${_DEFVAL}" STREQUAL "")

	SET(BUILD_fblib_${name} ${_DEFVAL} CACHE BOOL "Build the library fblib_${name}")
	IF(BUILD_fblib_${name}) 
	# --- Start of conditional build of module ---
	
	IF(NOT ${is_metalib})
		PROJECT(fblib_${name})
	ENDIF(NOT ${is_metalib})
	
	# There is an optional LISTS of extra sources from the caller: 
	#  "${name}_EXTRA_SRCS" and 
	#  "${name}_EXTRA_SRCS_NAME"   <--- Must NOT contain spaces!!
	#
	#  At return from this macro, there'll be defined a variable:
	#	   "${${name}_EXTRA_SRCS_NAME}_FILES"
	#   with the list of all files under that group.
	#
	#  For code simplicity, let's use the same list, just adding the default sources there:
	LIST(APPEND ${name}_EXTRA_SRCS 
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cpp"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.c"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cxx"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.h"
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/fblib/${name}/*.h"
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/fblib/${name}/*.hpp"
		"${CMAKE_SOURCE_DIR}/doc/doxygen-pages/lib_fblib_${name}.h"
		)
	LIST(APPEND ${name}_EXTRA_SRCS_NAME
		"${name}"
		"${name}"
		"${name}"
		"${name} Internal Headers"
		"${name} Public Headers"
		"${name} Public Headers"
		"Documentation"
		)
	# Only add these ones for "normal" libraries:
	IF (NOT ${headers_only})
		LIST(APPEND ${name}_EXTRA_SRCS 
			"${CMAKE_SOURCE_DIR}/libs/${name}/include/fblib/${name}/link_pragmas.h"
			)
		LIST(APPEND ${name}_EXTRA_SRCS_NAME
			"DLL link macros"
			)
	ENDIF (NOT ${headers_only})

	# Collect files
	# ---------------------------------------------------------
	LIST(LENGTH ${name}_EXTRA_SRCS N_SRCS)
	LIST(LENGTH ${name}_EXTRA_SRCS_NAME N_SRCS_NAMES)
	
	IF (NOT N_SRCS EQUAL N_SRCS_NAMES)
		MESSAGE(FATAL_ERROR "Mismatch length in ${name}_EXTRA_SRCS and ${name}_EXTRA_SRCS_NAME!")
	ENDIF (NOT N_SRCS EQUAL N_SRCS_NAMES)
	
	SET(${name}_srcs "")  # ALL the files
	
	MATH(EXPR N_SRCS "${N_SRCS}-1")  # Indices are 0-based
	
	foreach(i RANGE 0 ${N_SRCS})
		# Get i'th expression & its name:
		LIST(GET ${name}_EXTRA_SRCS      ${i} FILS_EXPR)
		LIST(GET ${name}_EXTRA_SRCS_NAME ${i} FILS_GROUP_NAME)
		
		FILE(GLOB aux_list ${FILS_EXPR})
		
		SOURCE_GROUP("${FILS_GROUP_NAME} files" FILES ${aux_list})
		
		# Add to main list:
		LIST(APPEND ${name}_srcs ${aux_list})
		# All to group lists, may be used by the user upon return from this macro:
		LIST(APPEND ${FILS_GROUP_NAME}_FILES ${aux_list})
	endforeach(i)

	# Remove _LIN files when compiling under Windows, and _WIN files when compiling under Linux.
	IF(WIN32)		
		REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" ${name}_srcs)		# Win32
	ELSE(WIN32)
		REMOVE_MATCHING_FILES_FROM_LIST(".*_WIN.cpp" ${name}_srcs)		# Apple & Unix
	ENDIF(WIN32)

	# Keep a list of unit testing files, for declaring them in /unittest:
	set(lstunittests ${${name}_srcs})
	KEEP_MATCHING_FILES_FROM_LIST(".*unittest.cpp" lstunittests)
	if(NOT "${lstunittests}" STREQUAL "")
		# We have unit tests:
		get_property(_lst_lib_test GLOBAL PROPERTY "FBLIB_TEST_LIBS")
		set_property(GLOBAL PROPERTY "FBLIB_TEST_LIBS" ${_lst_lib_test} fblib_${name})
		set_property(GLOBAL PROPERTY "fblib_${name}_UNIT_TEST_FILES" ${lstunittests})
	endif(NOT "${lstunittests}" STREQUAL "")


	# Don't include here the unit testing code:
	REMOVE_MATCHING_FILES_FROM_LIST(".*unittest.cpp" ${name}_srcs)


	#  Define the target:
	set(all_${name}_srcs  ${${name}_srcs})
	
	# Add main lib header (may not exist in meta-libs only):
	IF (EXISTS "${CMAKE_SOURCE_DIR}/libs/${name}/include/fblib/${name}.h")
		set(all_${name}_srcs ${all_${name}_srcs} "${CMAKE_SOURCE_DIR}/libs/${name}/include/fblib/${name}.h")
	ENDIF (EXISTS "${CMAKE_SOURCE_DIR}/libs/${name}/include/fblib/${name}.h")
		
	IF (NOT ${headers_only})

		# A libray target:
		ADD_LIBRARY(fblib_${name}   
			${all_${name}_srcs}      # sources
			${FBLIB_VERSION_RC_FILE}  # Only !="" in Win32: the .rc file with version info
			)

	ELSE(NOT ${headers_only})

		# A custom target (needs no real compiling)
		add_custom_target(fblib_${name} DEPENDS ${all_${name}_srcs} SOURCES ${all_${name}_srcs})

	ENDIF (NOT ${headers_only})

	# Append to list of all fblib_* libraries:
	if("${ALL_FBLIB_LIBS}" STREQUAL "")  # first one is different to avoid an empty first list element ";fblib_xxx"
		SET(ALL_FBLIB_LIBS "fblib_${name}" CACHE INTERNAL "")  # This emulates global vars
	else("${ALL_FBLIB_LIBS}" STREQUAL "")
		SET(ALL_FBLIB_LIBS "${ALL_FBLIB_LIBS};fblib_${name}" CACHE INTERNAL "")  # This emulates global vars
	endif("${ALL_FBLIB_LIBS}" STREQUAL "")
	
	# Include dir for this lib:
	INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/libs/${name}/include")
	
	# Include dirs for fblib_XXX libs:
	set(AUX_DEPS_LIST "")
	set(AUX_EXTRA_LINK_LIBS "")
	set(AUX_ALL_DEPS_BUILD 1)  # Will be set to "0" if any dependency if not built
	FOREACH(DEP ${ARGN})
		# Only for "fblib_XXX" libs:
		IF (${DEP} MATCHES "fblib_")
			STRING(REGEX REPLACE "fblib_(.*)" "\\1" DEP_FBLIB_NAME ${DEP})
			IF(NOT "${DEP_FBLIB_NAME}" STREQUAL "")
				# Include dir:
				INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/libs/${DEP_FBLIB_NAME}/include")
				
				# Link "-lfblib_name", only for GCC/CLang and if both THIS and the dependence are non-header-only:
				IF(NOT ${headers_only})
					IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
						get_property(_LIB_HDRONLY GLOBAL PROPERTY "${DEP}_LIB_IS_HEADERS_ONLY")
						IF(NOT _LIB_HDRONLY)
							#MESSAGE(STATUS "adding link dep: fblib_${name} -> ${DEP}")
							LIST(APPEND AUX_EXTRA_LINK_LIBS ${DEP}${FBLIB_LINKER_LIBS_POSTFIX})
						ENDIF(NOT _LIB_HDRONLY)
					ENDIF()
				ENDIF(NOT ${headers_only})
				
				# Append to list of fblib_* lib dependences:
				LIST(APPEND AUX_DEPS_LIST ${DEP})
				
				# Check if all dependencies are to be build: 
				if ("${BUILD_fblib_${DEP_FBLIB_NAME}}" STREQUAL "OFF")
					SET(AUX_ALL_DEPS_BUILD 0)
					MESSAGE(STATUS "*Warning*: Lib fblib_${name} cannot be built because dependency fblib_${DEP_FBLIB_NAME} has been disabled!")
				endif ()
				
			ENDIF(NOT "${DEP_FBLIB_NAME}" STREQUAL "")
		ENDIF (${DEP} MATCHES "fblib_")
	ENDFOREACH(DEP)
	
	# Impossible to build? 
	if (NOT AUX_ALL_DEPS_BUILD)
		MESSAGE(STATUS "*Warning* ==> Disabling compilation of lib fblib_${name} for missing dependencies listed above.")		
		SET(BUILD_fblib_${name} OFF CACHE BOOL "Build the library fblib_${name}" FORCE)
	endif (NOT AUX_ALL_DEPS_BUILD)
	
	
	# Emulates a global variable:
	set_property(GLOBAL PROPERTY "fblib_${name}_LIB_DEPS" "${AUX_DEPS_LIST}")
	set_property(GLOBAL PROPERTY "fblib_${name}_LIB_IS_HEADERS_ONLY" "${headers_only}")
	set_property(GLOBAL PROPERTY "fblib_${name}_LIB_IS_METALIB" "${is_metalib}")

	# Dependencies between projects:
	IF(NOT "${ARGN}" STREQUAL "")
		ADD_DEPENDENCIES(fblib_${name} ${ARGN})
	ENDIF(NOT "${ARGN}" STREQUAL "")

	IF (NOT ${headers_only})
		TARGET_LINK_LIBRARIES(fblib_${name} 
			${FBLIBLIB_LINKER_LIBS}
			${AUX_EXTRA_LINK_LIBS}
			)
	ENDIF (NOT ${headers_only})

	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(fblib_${name} PROPERTIES FOLDER "modules")
	else(ENABLE_SOLUTION_FOLDERS)
		SET_TARGET_PROPERTIES(fblib_${name} PROPERTIES PROJECT_LABEL "(LIB) fblib_${name}")
	endif(ENABLE_SOLUTION_FOLDERS)

	# Set custom name of lib + dynamic link numbering convenions in Linux:
	IF (NOT ${headers_only})
		SET_TARGET_PROPERTIES(fblib_${name} PROPERTIES 
			OUTPUT_NAME ${FBLIB_LIB_PREFIX}fblib_${name}${FBLIB_DLL_VERSION_POSTFIX}
			ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
			VERSION "${CMAKE_FBLIB_VERSION_NUMBER_MAJOR}.${CMAKE_FBLIB_VERSION_NUMBER_MINOR}.${CMAKE_FBLIB_VERSION_NUMBER_PATCH}"
			SOVERSION ${CMAKE_FBLIB_VERSION_NUMBER_MAJOR}.${CMAKE_FBLIB_VERSION_NUMBER_MINOR}
			)
		
		# Set all header files as "ignored" (don't build!):
		# -----------------------------------------------------
		set(AUX_LIST_TO_IGNORE ${all_${name}_srcs})
		KEEP_MATCHING_FILES_FROM_LIST("^.*h$" AUX_LIST_TO_IGNORE)
		set_source_files_properties(${AUX_LIST_TO_IGNORE} PROPERTIES HEADER_FILE_ONLY true)
	
		INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/libs/${name}/src/") # For include "${name}_precomp.h"
		IF(FBLIB_ENABLE_PRECOMPILED_HDRS)
			IF (MSVC)
				# Precompiled hdrs for MSVC:
				# --------------------------------------
				STRING(TOUPPER ${name} NAMEUP)

				# The "use precomp.headr" for all the files...
				set_target_properties(fblib_${name}
					PROPERTIES
					COMPILE_FLAGS "/Yu${name}_precomp.h")

				# But for the file used to build the precomp. header:
				set_source_files_properties("${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}_precomp.cpp"
					PROPERTIES
					COMPILE_FLAGS "/Yc${name}_precomp.h")
			ENDIF (MSVC)
		
			SOURCE_GROUP("Precompiled headers" FILES 
				"${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}_precomp.cpp"
				"${CMAKE_SOURCE_DIR}/libs/${name}/include/${name}_precomp.h"
				)	
		ENDIF(FBLIB_ENABLE_PRECOMPILED_HDRS)

		# Special directories when building a .deb package:
		IF(CMAKE_FBLIB_USE_DEB_POSTFIXS)
			SET(FBLIB_PREFIX_INSTALL "${CMAKE_INSTALL_PREFIX}/libfblib_${name}${CMAKE_FBLIB_VERSION_NUMBER_MAJOR}.${CMAKE_FBLIB_VERSION_NUMBER_MINOR}/usr/")
		ELSE(CMAKE_FBLIB_USE_DEB_POSTFIXS)
			SET(FBLIB_PREFIX_INSTALL "")
		ENDIF(CMAKE_FBLIB_USE_DEB_POSTFIXS)

		# make sure the library gets installed
		IF (NOT is_metalib)
			INSTALL(TARGETS fblib_${name}
				RUNTIME DESTINATION ${FBLIB_PREFIX_INSTALL}bin  COMPONENT Libraries
				LIBRARY DESTINATION ${FBLIB_PREFIX_INSTALL}${CMAKE_INSTALL_LIBDIR} COMPONENT Libraries
				ARCHIVE DESTINATION ${FBLIB_PREFIX_INSTALL}${CMAKE_INSTALL_LIBDIR} COMPONENT Libraries  # WAS: lib${LIB_SUFFIX}
				)
			
			# Collect .pdb debug files for optional installation:
			IF (MSVC)
				SET(PDB_FILE "${CMAKE_BINARY_DIR}/bin/Debug/fblib_${name}${CMAKE_FBLIB_VERSION_NUMBER_MAJOR}${CMAKE_FBLIB_VERSION_NUMBER_MINOR}${CMAKE_FBLIB_VERSION_NUMBER_PATCH}d.pdb")
				IF (EXISTS "${PDB_FILE}")
					INSTALL(FILES ${PDB_FILE} DESTINATION bin COMPONENT LibrariesDebugInfoPDB)
				ENDIF (EXISTS "${PDB_FILE}")
			ENDIF(MSVC)		
		ENDIF (NOT is_metalib)
	ENDIF (NOT ${headers_only})

	# Generate the libfblib_$NAME.pc file for pkg-config:
	IF(UNIX)
		SET(fblib_pkgconfig_LIBNAME ${name})
		get_property(_lst_deps GLOBAL PROPERTY "fblib_${name}_LIB_DEPS")
		
		# a comma-separated list of other fblib_* dependencies.
		SET(fblib_pkgconfig_REQUIRES "")
		FOREACH(DEP ${_lst_deps})
			IF(NOT "${fblib_pkgconfig_REQUIRES}" STREQUAL "")
				SET(fblib_pkgconfig_REQUIRES "${fblib_pkgconfig_REQUIRES},")
			ENDIF(NOT "${fblib_pkgconfig_REQUIRES}" STREQUAL "")
			SET(fblib_pkgconfig_REQUIRES "${fblib_pkgconfig_REQUIRES}${DEP}")
		ENDFOREACH(DEP)

		# Special case: For fblib_base, mark "eigen3" as a pkg-config dependency only 
		#  if we are instructed to do so: (EIGEN_USE_EMBEDDED_VERSION=OFF)
		IF(NOT EIGEN_USE_EMBEDDED_VERSION)
			SET(fblib_pkgconfig_REQUIRES "${fblib_pkgconfig_REQUIRES},eigen3")
		ENDIF(NOT EIGEN_USE_EMBEDDED_VERSION)

		# "Libs" lines in .pc files:
		# -----------------------------------------------------------
		# * for install, normal lib:
		#    Libs: -L${libdir}  -lfblib_@fblib_pkgconfig_LIBNAME@ 
		# * for install, headers-only lib:
		#    <none>
		# * for local usage, normal lib:
		#    Libs: -L${libdir} -Wl,-rpath,${libdir} -lfblib_@fblib_pkgconfig_LIBNAME@ 
		# * for local usage, headers-only lib:
		#    <none>
		IF (${headers_only})
			SET(fblib_pkgconfig_lib_line_install "")
			SET(fblib_pkgconfig_lib_line_noinstall "")
			SET(fblib_pkgconfig_libs_private_line "")
		ELSE (${headers_only})
			SET(fblib_pkgconfig_lib_line_install "Libs: -L\${libdir}  -lfblib_${name}")
			SET(fblib_pkgconfig_lib_line_noinstall "Libs: -L\${libdir} -Wl,-rpath,\${libdir} -lfblib_${name}")
			SET(fblib_pkgconfig_libs_private_line "Libs.private: ${FBLIBLIB_LINKER_LIBS}")
		ENDIF (${headers_only})

		# (1/2) Generate the .pc file for "make install"
		CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/fblib_template.pc.in" "${CMAKE_BINARY_DIR}/pkgconfig/fblib_${name}.pc" @ONLY)

		# (2/2) And another .pc file for local usage:
		SET(fblib_pkgconfig_NO_INSTALL_SOURCE "${FBLIB_SOURCE_DIR}")
		SET(fblib_pkgconfig_NO_INSTALL_BINARY "${FBLIB_BINARY_DIR}")
		CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/fblib_template_no_install.pc.in" "${CMAKE_BINARY_DIR}/pkgconfig-no-install/fblib_${name}.pc" @ONLY)
		
	ENDIF(UNIX)

	# --- End of conditional build of module ---
	ENDIF(BUILD_fblib_${name}) 

endmacro(internal_define_fblib_lib)

