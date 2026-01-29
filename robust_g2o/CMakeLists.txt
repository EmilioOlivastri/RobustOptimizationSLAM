set(G2O_ROOT /home/slamemix/Workspace/libs/g2o)

find_package(G2O REQUIRED)
find_package (glog)
find_package(yaml-cpp REQUIRED)

add_subdirectory(thirdParty)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  "/usr/local/include/eigen3"
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
  yaml-cpp::yaml-cpp
)

target_link_libraries(g2o_MAXMIX_3D
  maxmixlib  
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  glog
  yaml-cpp::yaml-cpp
)

add_executable(g2o_SC_2D
src/sc_2D.cpp
src/utils.cpp
)

target_link_libraries(g2o_SC_2D
${G2O_TYPES_SLAM2D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
yaml-cpp::yaml-cpp
${GLOG_LIBRARY}
switchlib
)

add_executable(g2o_SC_3D
src/sc_3D.cpp
src/utils.cpp
)

target_link_libraries(g2o_SC_3D
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
yaml-cpp::yaml-cpp
${GLOG_LIBRARY}
switchlib
)

add_executable(g2o_RRR_2D
src/rrr_2D.cpp
src/utils.cpp
)

target_link_libraries(g2o_RRR_2D
${G2O_TYPES_SLAM2D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
yaml-cpp::yaml-cpp
${GLOG_LIBRARY}
rrrlib
)

add_executable(g2o_RRR_3D
src/rrr_3D.cpp
src/utils.cpp
)

target_link_libraries(g2o_RRR_3D
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
yaml-cpp::yaml-cpp
${GLOG_LIBRARY}
rrrlib
)


add_executable(g2o_GNC_2D
src/gnc_2D.cpp
src/utils.cpp
)

target_link_libraries(g2o_GNC_2D
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
yaml-cpp::yaml-cpp
${GLOG_LIBRARY}
gncg2olib
)


add_executable(g2o_GNC_3D
src/gnc_3D.cpp
src/utils.cpp
)

target_link_libraries(g2o_GNC_3D
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
yaml-cpp::yaml-cpp
${GLOG_LIBRARY}
gncg2olib
)

add_executable(g2o_dataset_enhancer 
src/dataset_enhancer.cpp 
src/utils.cpp)


target_link_libraries(g2o_dataset_enhancer
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  glog
  yaml-cpp::yaml-cpp
)

add_executable(g2o_dataset_seeder 
src/artificial_seeder.cpp 
src/utils.cpp)


target_link_libraries(g2o_dataset_seeder
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  glog
  yaml-cpp::yaml-cpp
)


add_executable(g2o_noise_applier_2D 
src/noise_applier_2D.cpp 
src/utils.cpp)

target_link_libraries(g2o_noise_applier_2D
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  glog
  yaml-cpp::yaml-cpp
)