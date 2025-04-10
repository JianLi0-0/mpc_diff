cmake_minimum_required(VERSION 3.0.2)
project(mpc_diff)

message(status "IPOPT_CFLAGS: " ${IPOPT_CFLAGS})
message(status "IPOPT_CFLAGS_OTHER: " ${IPOPT_CFLAGS_OTHER})
set(CMAKE_CXX_FLAGS "-DHAVE_CSTDDEF -DHAVE_MPI_INITIALIZED")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

find_package(trajectory_utils REQUIRED)

find_package(Python3 REQUIRED COMPONENTS
        Interpreter
        Development
        NumPy
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mpc_diff

        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        /usr/local/include
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        /usr/local/include/osqp
)
link_directories(/usr/local/lib)

add_definitions(${EIGEN3_DEFINITIONS})

add_executable(traj_server
        src/traj_server.cpp
        src/MPC.cpp)

target_link_libraries(traj_server
        ${catkin_LIBRARIES}
        trajectory_utils
#        libqpOASES.so
        Python3::Python
        Python3::Module
        Python3::NumPy
        /usr/local/lib/libosqp.so
        /usr/local/lib/libOsqpEigen.so
)
