if(GTSAM_BUILD_NAME)
  set(gtsam_build_names "${GTSAM_BUILD_NAME}/gtsam")
else()
  # lowercase build type
  string(TOLOWER "${CMAKE_BUILD_TYPE}" build_type_suffix)
  # build suffix of this project
  get_filename_component(my_build_name "${CMAKE_BINARY_DIR}" NAME)
  
  set(gtsam_build_names "${my_build_name}/gtsam" "build-${build_type_suffix}/gtsam" "build/gtsam")
endif()

# Use GTSAM_ROOT or GTSAM_DIR equivalently
if(GTSAM_ROOT AND NOT GTSAM_DIR)
  set(GTSAM_DIR "${GTSAM_ROOT}")
endif()

if(GTSAM_DIR)
  # Find include dirs
  find_path(GTSAM_INCLUDE_DIR gtsam/inference/FactorGraph.h
    PATHS "${GTSAM_DIR}/include" "${GTSAM_DIR}"
          "/home/slam-emix/Workspace/lib/gtsam"
          NO_DEFAULT_PATH
    DOC "GTSAM include directories")

  # Find libraries
  find_library(GTSAM_LIBS NAMES gtsam
    HINTS "${GTSAM_DIR}/lib" "${GTSAM_DIR}"
          "/home/slam-emix/Workspace/lib/gtsam/build/gtsam" 
          NO_DEFAULT_PATH
    PATH_SUFFIXES ${gtsam_build_names}
    DOC "GTSAM libraries")
else()
  # Find include dirs
  set(extra_include_paths ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" "${PROJECT_SOURCE_DIR}/../gtsam" /usr/local/include /usr/include)
  find_path(GTSAM_INCLUDE_DIR gtsam/inference/FactorGraph.h
    PATHS ${extra_include_paths}
    DOC "GTSAM include directories")
  if(NOT GTSAM_INCLUDE_DIR)
    message(STATUS "Searched for gtsam headers in default paths plus ${extra_include_paths}")
  endif()

  # Find libraries
  find_library(GTSAM_LIBS NAMES gtsam
    HINTS ${CMAKE_INSTALL_PREFIX}/lib 
          "$ENV{HOME}/lib" "${PROJECT_SOURCE_DIR}/../gtsam" 
          "/usr/local/lib" "/usr/lib" 
          "/home/slam-emix/Workspace/lib/gtsam/build/gtsam"
    PATH_SUFFIXES ${gtsam_build_names}
    DOC "GTSAM libraries")
endif()

message("GTSAM INCLUDE DIR = ${GTSAM_DIR}")
message("GTSAM LIBS DIR = ${GTSAM_LIBS}")


# handle the QUIETLY and REQUIRED arguments and set GTSAM_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTSAM DEFAULT_MSG
GTSAM_INCLUDE_DIR GTSAM_LIBS)
 
