find_package(MKL REQUIRED)
find_package(TBB REQUIRED)
find_package(KimeraRPGO REQUIRED)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${KimeraRPGO_INCLUDE_DIR}
  "/usr/local/include/eigen3"
  ${YAML_CPP_INCLUDE_DIR}
)

message(STATUS "GTSAM LIBS: ${GTSAM_LIBS}")

add_executable(gtsam_HUBER_2D src/huber_2D.cpp src/utils.cpp)
add_executable(gtsam_HUBER_3D src/huber_3D.cpp src/utils.cpp)
add_executable(gtsam_DCS_2D src/dcs_2D.cpp src/utils.cpp)
add_executable(gtsam_DCS_3D src/dcs_3D.cpp src/utils.cpp)
add_executable(gtsam_GNC_2D src/gnc_2D.cpp src/utils.cpp)
add_executable(gtsam_GNC_3D src/gnc_3D.cpp src/utils.cpp)
add_executable(gtsam_PCM_2D src/rpgo_2D.cpp src/utils.cpp)
add_executable(gtsam_PCM_3D src/rpgo_3D.cpp src/utils.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_HUBER_2D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_HUBER_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_GNC_2D
gtsam
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_GNC_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_DCS_2D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_DCS_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_PCM_2D
gtsam
tbb
tbbmalloc
KimeraRPGO
${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_PCM_3D
gtsam
tbb
tbbmalloc
KimeraRPGO
${YAML_CPP_LIBRARIES}
)
