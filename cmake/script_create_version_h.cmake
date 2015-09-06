# ----------------------------------------------------------------------------
#  Update the library version header file
#    FILE_TO_PARSE="SRC/include/openmvo/OPENMVO_version.h.in"
#    TARGET_FILE  ="OPENMVO_version.h"
# ----------------------------------------------------------------------------
SET(CMAKE_OPENMVO_COMPLETE_NAME "OPENMVO ${CMAKE_OPENMVO_VERSION_NUMBER_MAJOR}.${CMAKE_OPENMVO_VERSION_NUMBER_MINOR}.${CMAKE_OPENMVO_VERSION_NUMBER_PATCH}")
# Build a three digits version code, eg. 0.5.1 -> 051,  1.2.0 -> 120
SET(CMAKE_OPENMVO_VERSION_CODE "0x${CMAKE_OPENMVO_VERSION_NUMBER_MAJOR}${CMAKE_OPENMVO_VERSION_NUMBER_MINOR}${CMAKE_OPENMVO_VERSION_NUMBER_PATCH}")

CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/version.h.in" "${OPENMVO_CONFIG_FILE_INCLUDE_DIR}/openmvo/version.h")

