set(G2O_ROOT /home/slam-emix/Workspace/lib/g2o)

find_package(G2O REQUIRED)
find_package (glog)

add_subdirectory(thirdParty)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  "/usr/include/eigen3"
  ${G2O_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  thirdParty
)

add_executable(g2o_MAXMIX_2D
src/maxmix_2D.cpp  
src/utils.cpp)
add_executable(g2o_MAXMIX_3D 
src/maxmix_3D.cpp 
src/utils.cpp)


target_link_libraries(g2o_MAXMIX_2D
  maxmixlib  
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  glog
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(g2o_MAXMIX_3D
  maxmixlib  
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  glog
  ${YAML_CPP_LIBRARIES}
)

add_executable(g2o_sc_2D
src/sc_2D.cpp
src/utils.cpp
)

target_link_libraries(g2o_sc_2D
${G2O_TYPES_SLAM2D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
${YAML_CPP_LIBRARIES}
${GLOG_LIBRARY}
switchlib
)

add_executable(g2o_sc_3D
src/sc_3D.cpp
src/utils.cpp
)

target_link_libraries(g2o_sc_3D
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
${YAML_CPP_LIBRARIES}
${GLOG_LIBRARY}
switchlib
)