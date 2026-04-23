cmake_minimum_required(VERSION 3.10)
project(test_motor_thread)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 REQUIRED COMPONENTS Development)
find_package(jsoncpp REQUIRED)

add_executable(test_motor_thread main.cpp)

target_include_directories(test_motor_thread PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/thirdparty/include/ddscxx
    ${Python3_INCLUDE_DIRS}
)

# unitree_sdk2 (provides Thread, RecurrentThread, etc.)
add_library(unitree_sdk2 STATIC IMPORTED)
set_target_properties(unitree_sdk2 PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libunitree_sdk2.a)

# ddsc / ddscxx (unitree_sdk2 depends on these)
add_library(ddsc SHARED IMPORTED)
set_target_properties(ddsc PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libddsc.so
    IMPORTED_NO_SONAME TRUE)

add_library(ddscxx SHARED IMPORTED)
set_target_properties(ddscxx PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libddscxx.so
    IMPORTED_NO_SONAME TRUE)

# motor SDK
find_library(UNITREE_MOTOR_SDK UnitreeMotorSDK_Linux64
    PATHS ${PROJECT_SOURCE_DIR}/lib NO_DEFAULT_PATH)

target_link_libraries(test_motor_thread
    unitree_sdk2 ddsc ddscxx
    ${UNITREE_MOTOR_SDK}
    Python3::Python
    JsonCpp::JsonCpp
    pthread rt
)
