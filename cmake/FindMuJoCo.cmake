set(_MUJOCO_ROOTS "")
if(DEFINED ENV{MUJOCO_HOME} AND NOT "$ENV{MUJOCO_HOME}" STREQUAL "")
  list(APPEND _MUJOCO_ROOTS "$ENV{MUJOCO_HOME}")
endif()
list(APPEND _MUJOCO_ROOTS "$ENV{HOME}/.mujoco/mujoco-3.4.0" "$ENV{HOME}/.mujoco")

find_path(
  MUJOCO_INCLUDE_DIR
  mujoco/mujoco.h
  PATHS ${_MUJOCO_ROOTS}
  PATH_SUFFIXES include
)
find_library(
  MUJOCO_LIBRARY
  NAMES mujoco
  PATHS ${_MUJOCO_ROOTS}
  PATH_SUFFIXES lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  MuJoCo
  REQUIRED_VARS MUJOCO_INCLUDE_DIR MUJOCO_LIBRARY
)

if(MUJOCO_FOUND AND NOT TARGET MuJoCo::MuJoCo)
  add_library(MuJoCo::MuJoCo UNKNOWN IMPORTED)
  set_target_properties(
    MuJoCo::MuJoCo
    PROPERTIES
      IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
  )
endif()
