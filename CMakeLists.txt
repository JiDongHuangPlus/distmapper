cmake_minimum_required(VERSION 3.0.2)
project(distributed_mapper)
add_compile_options(-std=c++11)

find_package(GTSAM REQUIRED)
find_package(Boost COMPONENTS serialization filesystem system program_options thread REQUIRED)

include_directories(
# include
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib distributed_mapper.cpp types.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${Boost_LIBRARIES} gtsam)

add_library(centralized_lib centralized_mapper.cpp types.cpp)
target_link_libraries(centralized_lib ${Boost_LIBRARIES} gtsam)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib centralized_lib)