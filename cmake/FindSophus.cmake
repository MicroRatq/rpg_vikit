# Fallback FindSophus.cmake for environments where SophusConfig.cmake is absent.
# This is useful if Sophus is installed as legacy headers+lib (e.g. /usr/local/include, /usr/local/lib).

find_path(Sophus_INCLUDE_DIRS
  NAMES sophus/so3.h
  PATHS /usr/local/include /usr/include
)

find_library(Sophus_LIBRARIES
  NAMES Sophus libSophus
  PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Sophus DEFAULT_MSG Sophus_INCLUDE_DIRS Sophus_LIBRARIES)

if(Sophus_FOUND AND NOT TARGET Sophus::Sophus)
  add_library(Sophus::Sophus SHARED IMPORTED)
  set_target_properties(Sophus::Sophus PROPERTIES
    IMPORTED_LOCATION "${Sophus_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${Sophus_INCLUDE_DIRS}"
  )
endif()

