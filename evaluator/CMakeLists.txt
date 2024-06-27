include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  "/usr/include/eigen3"
)

add_executable(evaluator src/evaluator.cpp src/metric_eval.cpp)