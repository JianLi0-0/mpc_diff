cmake_minimum_required(VERSION 3.0.2)
project(mpc_diff)

message(status "IPOPT_CFLAGS: " ${IPOPT_CFLAGS})
message(status "IPOPT_CFLAGS_OTHER: " ${IPOPT_CFLAGS_OTHER})
set(CMAKE_CXX_FLAGS "-DHAVE_CSTDDEF -DHAVE_MPI_INITIALIZED")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp

        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mpc_diff

        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

add_library(mpc_diff
        src/MPC.cpp
        include/MPC.hpp)
target_link_libraries(mpc_diff
        ${catkin_LIBRARIES}
        )

include_directories(
        include
        /usr/local/include
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)
link_directories(/usr/local/lib)

add_definitions(${EIGEN3_DEFINITIONS})

add_executable(traj_server src/traj_server.cpp
                           src/uniform_bspline.cpp
                           src/MPC.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES} libqpOASES.so)

add_executable(test_path_pub src/test_path_pub.cpp)
target_link_libraries(test_path_pub ${catkin_LIBRARIES})
