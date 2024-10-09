cmake_minimum_required(VERSION 3.5.0)
project(tracking_library)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/deps/eigen/cmake")
set(EIGEN3_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/deps/eigen/")
set(EIGEN3_INCLUDE_DIR "${EIGEN3_INCLUDE_DIRS}")
find_package(Eigen3 REQUIRED)

set(manif_DIR "${CMAKE_CURRENT_LIST_DIR}/deps/manif/install/share/manif/cmake")
find_package(manif REQUIRED)
include_directories(${manif_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(ekf_pose3_tracker SHARED
  src/ekf_pose3_tracker.cpp
  )

add_library(tracking_library SHARED
  src/tracking_library.cpp
  src/oculus_original_tracking.cpp
  src/statistics_tracker.cpp
  src/constrained_complementary_tracking.cpp
  src/directed_complementary_tracking.cpp
  src/complementary_tracking.cpp
  src/double_exponential_smoothing.cpp
  src/exponential_smoothing.cpp
  src/particle_filter.cpp
  src/tracking_commons.cpp
  src/kalman_tracking.cpp
  src/kalman_tracking_wrapper.cpp
  src/kalman_tracking_constrained.cpp
  src/tracking_parser_library.cpp
  )

target_link_libraries(tracking_library
  ekf_pose3_tracker
  )
  
install(TARGETS tracking_library
        CONFIGURATIONS Debug Release RelWithDebInfo
        LIBRARY DESTINATION ${CMAKE_CURRENT_LIST_DIR}/Plugins/x64)
        
add_executable(tracking_test
  src/tracking_test.cpp
  )
  
target_link_libraries(tracking_test 
  tracking_library
  )
  
add_executable(tracking_compute_stats
  src/tracking_compute_stats.cpp
  )
  
target_link_libraries(tracking_compute_stats 
  tracking_library
  )

add_executable(tracking_parser
  src/tracking_parser.cpp
  )

target_link_libraries(tracking_parser 
  tracking_library
  )

add_executable(tracking_parser_compute_errors
  src/tracking_parser_compute_errors.cpp
  )

target_link_libraries(tracking_parser_compute_errors 
  tracking_library
  )

add_executable(tracking_parser_extract_matrices
  src/tracking_parser_extract_matrices.cpp
  )

target_link_libraries(tracking_parser_extract_matrices
  tracking_library
  )

add_executable(tracking_parser_matrix_extract_translation
  src/tracking_parser_matrix_extract_translation.cpp
  )

target_link_libraries(tracking_parser_matrix_extract_translation
  tracking_library
  )

add_executable(tracking_parser_get_sequence
  src/tracking_parser_get_sequence.cpp
  )

target_link_libraries(tracking_parser_get_sequence 
  tracking_library
  )

add_executable(tracking_parser_merge_error_db
  src/tracking_parser_merge_error_db.cpp
  )

target_link_libraries(tracking_parser_merge_error_db
  tracking_library
  )
