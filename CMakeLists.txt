cmake_minimum_required(VERSION 2.8)
project(learn_SLAM)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED COMPONENT common io)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE ALL_SRCS "*.cc")
foreach(FILE_PATH ${ALL_SRCS})
  get_filename_component(DIRECTORY_PATH ${FILE_PATH} PATH)
  get_filename_component(DIRECTORY_NAME ${DIRECTORY_PATH} NAME)
  get_filename_component(FILENAME ${FILE_PATH} NAME_WE)
  set(EXEC "${DIRECTORY_NAME}${FILENAME}")
  add_executable(${EXEC} ${FILE_PATH})
  target_link_libraries(${EXEC} ${Sophus_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
endforeach(FILE_PATH)
