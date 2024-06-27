set(G2O_ROOT /home/slam-emix/Workspace/lib/g2o)

find_package(G2O REQUIRED)
find_package (glog)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  "/usr/include/eigen3"
  ${G2O_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(g2o_MAXMIX 
src/maxmix.cpp 
src/edge_se2_mixture.cpp 
src/edge_se3_mixture.cpp 
src/utils.cpp)


target_link_libraries(g2o_MAXMIX
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  glog
  ${YAML_CPP_LIBRARIES}
)
