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


add_executable(gtsam_HUBER src/huber.cpp src/utils.cpp)
add_executable(gtsam_GNC src/gnc.cpp src/utils.cpp)
add_executable(gtsam_DCS src/dcs.cpp src/utils.cpp)
add_executable(gtsam_PCM src/rpgo.cpp src/utils.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_HUBER
${catkin_LIBRARIES}
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_GNC
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_DCS
${GTSAM_LIBS}
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_PCM
${GTSAM_LIBS}
tbb
tbbmalloc
KimeraRPGO
${YAML_CPP_LIBRARIES}
)
