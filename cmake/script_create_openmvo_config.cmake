# ----------------------------------------------------------------------------
#   Generate the OPENMVOConfig.cmake & configure files
# ----------------------------------------------------------------------------
# Create the code fragment: "DECLARE_LIBS_DEPS", for usage below while
#  generating "OPENMVOConfig.cmake"
SET(DECLARE_LIBS_DEPS "")
FOREACH(_LIB ${ALL_OPENMVO_LIBS})
	get_property(_LIB_DEP GLOBAL PROPERTY "${_LIB}_LIB_DEPS")
	SET(DECLARE_LIBS_DEPS "${DECLARE_LIBS_DEPS} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_DEPS\" \"${_LIB_DEP}\")\n")
ENDFOREACH(_LIB)

# Create the code fragment: "DECLARE_LIBS_HDR_ONLY", for usage below while
#  generating "OPENMVOConfig.cmake"
SET(DECLARE_LIBS_HDR_ONLY "")
FOREACH(_LIB ${ALL_OPENMVO_LIBS})
	get_property(_LIB_HDR_ONLY GLOBAL PROPERTY "${_LIB}_LIB_IS_HEADERS_ONLY")
	SET(DECLARE_LIBS_HDR_ONLY "${DECLARE_LIBS_HDR_ONLY} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_IS_HEADERS_ONLY\" \"${_LIB_HDR_ONLY}\")\n")
ENDFOREACH(_LIB)

# ----------------------------------------------------------------------------
#   Generate the OPENMVOConfig.cmake file
# ----------------------------------------------------------------------------
SET(THE_OPENMVO_SOURCE_DIR "${OPENMVO_SOURCE_DIR}")
SET(THE_OPENMVO_LIBS_INCL_DIR "${THE_OPENMVO_SOURCE_DIR}/libs")
SET(THE_CMAKE_BINARY_DIR "${CMAKE_BINARY_DIR}")
SET(THE_OPENMVO_CONFIG_FILE_INCLUDE_DIR "${OPENMVO_CONFIG_FILE_INCLUDE_DIR}")
SET(OPENMVO_CONFIGFILE_IS_INSTALL 0)

CONFIGURE_FILE(
	"${OPENMVO_SOURCE_DIR}/parse-files/OPENMVOConfig.cmake.in"
    "${OPENMVO_BINARY_DIR}/OPENMVOConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding OPENMVO, e.g. find_package(OPENMVO 1.0.0 EXACT)
CONFIGURE_FILE(
	"${OPENMVO_SOURCE_DIR}/parse-files/OPENMVOConfig-version.cmake.in" 
	"${CMAKE_BINARY_DIR}/OPENMVOConfig-version.cmake" IMMEDIATE @ONLY)

# ----------------------------------------------------------------------------
#   Generate the OPENMVOConfig.cmake file for unix
#      installation in CMAKE_INSTALL_PREFIX
# ----------------------------------------------------------------------------
SET(OPENMVO_CONFIGFILE_IS_INSTALL 1)
IF(WIN32)
	SET(THE_OPENMVO_SOURCE_DIR "\${THIS_OPENMVO_CONFIG_PATH}")
	SET(THE_OPENMVO_LIBS_INCL_DIR "${THE_OPENMVO_SOURCE_DIR}/libs")
	SET(THE_CMAKE_BINARY_DIR "\${THIS_OPENMVO_CONFIG_PATH}")
	SET(THE_OPENMVO_CONFIG_FILE_INCLUDE_DIR "\${THIS_OPENMVO_CONFIG_PATH}/include/fblib/fblib_config/")
ELSE(WIN32)
	# Unix install. This .cmake file will end up in /usr/share/fblib/OPENMVOConfig.cmake :
	IF (CMAKE_OPENMVO_USE_DEB_POSTFIXS)
		# We're building a .deb package: DESTDIR is NOT the final installation directory:
		SET(THE_OPENMVO_SOURCE_DIR "/usr")
		SET(THE_OPENMVO_LIBS_INCL_DIR "${THE_OPENMVO_SOURCE_DIR}/include/fblib")
		SET(THE_CMAKE_BINARY_DIR "/usr")
		SET(THE_OPENMVO_CONFIG_FILE_INCLUDE_DIR "/usr/include/fblib/fblib_config/")
	ELSE(CMAKE_OPENMVO_USE_DEB_POSTFIXS)
		# Normal case: take the desired installation directory
		SET(THE_OPENMVO_SOURCE_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_OPENMVO_LIBS_INCL_DIR "${THE_OPENMVO_SOURCE_DIR}/include/fblib")
		SET(THE_CMAKE_BINARY_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_OPENMVO_CONFIG_FILE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/fblib/fblib_config/")
	ENDIF(CMAKE_OPENMVO_USE_DEB_POSTFIXS)
ENDIF(WIN32)

CONFIGURE_FILE(
	"${OPENMVO_SOURCE_DIR}/parse-files/OPENMVOConfig.cmake.in"  
	"${OPENMVO_BINARY_DIR}/unix-install/OPENMVOConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding OPENMVO, e.g. find_package(OPENMVO 1.0.0 EXACT)
CONFIGURE_FILE(
	"${OPENMVO_SOURCE_DIR}/parse-files/OPENMVOConfig-version.cmake.in" 
	"${OPENMVO_BINARY_DIR}/unix-install/OPENMVOConfig-version.cmake" IMMEDIATE @ONLY)
