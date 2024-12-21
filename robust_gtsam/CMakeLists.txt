set(GTSAM_DIR /home/slam-emix/Workspace/lib/gtsam/build)

find_package(GTSAM REQUIRED)
find_package(MKL REQUIRED)
find_package(TBB REQUIRED)
find_package(KimeraRPGO REQUIRED)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${KimeraRPGO_INCLUDE_DIR}
  "/usr/include/eigen3"
  ${YAML_CPP_INCLUDE_DIR}
)

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
${catkin_LIBRARIES}
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_HUBER_3D
${catkin_LIBRARIES}
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_GNC_2D
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_GNC_3D
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_DCS_2D
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_DCS_3D
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_PCM_2D
${GTSAM_LIBS}
tbb
tbbmalloc
KimeraRPGO
${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_PCM_3D
${GTSAM_LIBS}
tbb
tbbmalloc
KimeraRPGO
${YAML_CPP_LIBRARIES}
)
