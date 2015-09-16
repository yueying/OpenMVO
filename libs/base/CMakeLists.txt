macro(add_base_srcs _PATH)
	LIST(APPEND base_EXTRA_SRCS		"${OPENMVO_LIBS_ROOT}/base/src/${_PATH}/*.cpp" 
	"${OPENMVO_LIBS_ROOT}/base/include/openmvo/${_PATH}/*.h" 
	"${OPENMVO_LIBS_ROOT}/base/include/openmvo/${_PATH}/*.hpp" 
	)
	LIST(APPEND base_EXTRA_SRCS_NAME 	"${_PATH}" "${_PATH}" "${_PATH}")
endmacro(add_base_srcs)

add_base_srcs(utils)
add_base_srcs(math)

#ÃÌº”±‡“Î÷∏¡Ó
ADD_DEFINITIONS(-D__SSE2__)#∆Ù”√SSE2
#---------------------------------------------
# Macro declared in "DeclareOPENMVOLib.cmake":
#---------------------------------------------
define_openmvo_lib(
	# Lib name
	base 		
	# Dependencies
	)
