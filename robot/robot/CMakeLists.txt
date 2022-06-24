cmake_minimum_required(VERSION 3.12)

project(robot)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

add_executable(robot src/main.cpp src/robot.cpp src/robot.h src/control_server.h src/control_server.cpp include/general.hpp include/CubicSplineNoDynamic.cpp include/CubicSplineNoDynamic.h include/GetPosIK.cpp include/GetPosIK.h include/Param.h include/TransMechPositionIK.h include/TransMechPositionIK.cpp include/WalkLegExeLegIk.cpp include/WalkLegExeLegIk.h)

target_include_directories(robot PUBLIC ${kaanh_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(robot ${kaanh_LIBRARIES} ${aris_LIBRARIES} atomic pthread stdc++fs)

if(UNIX)
    target_link_libraries(robot ${aris_LIBRARIES} atomic modbus /usr/local/lib/libmodbus.so.5)
endif(UNIX)
IF(WIN32)
    target_link_libraries(robot ${aris_LIBRARIES})
endif(WIN32)

