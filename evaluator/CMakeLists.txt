set(G2O_ROOT /home/slam-emix/Workspace/lib/g2o)

find_package(G2O REQUIRED)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  "/usr/include/eigen3"
  ${G2O_INCLUDE_DIR}
)

add_executable(evaluator src/evaluator.cpp src/metric_eval.cpp)
add_executable(test_newmetrics src/test_new_metrics.cpp src/metric_eval.cpp)


target_link_libraries(evaluator
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
)

target_link_libraries(test_newmetrics
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
)