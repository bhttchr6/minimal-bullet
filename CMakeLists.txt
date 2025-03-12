# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Project Name
project(Robotics_Project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set paths to GLFW and GLAD and SDL
set(GLFW_DIR "${CMAKE_SOURCE_DIR}/third/glfw")
set(GLAD_DIR "${CMAKE_SOURCE_DIR}/third/glad")
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(SDL_DIR "${CMAKE_SOURCE_DIR}/third/vendored")
set(Bullet_DIR "${CMAKE_SOURCE_DIR}/third/bullet3/build_cmake")
set(BULLET_LIBRARY_DIRS "${Bullet_DIR}/lib")

message("Finding local Bullet")
find_package(Bullet REQUIRED CONFIG)

if (Bullet_FOUND)
    message(STATUS "Bullet found: ${Bullet_DIR}")
    set(BULLET_INCLUDE_DIR ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
    set(BULLET_LIBRARY_DIRS ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
else()
    message(FATAL_ERROR "Bullet Physics SDK not found!")
endif()
#message("Finding local Bullet")
#Find_Package(Bullet CONFIG)
#SET(BULLET_INCLUDE_DIR ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
#SET(BLA ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
MESSAGE("BULLET_INCLUDE_DIR:")
MESSAGE(${BULLET_INCLUDE_DIR})
MESSAGE("Bullet Libraries:")
MESSAGE(${BULLET_LIBRARIES})
MESSAGE("BULLET_LIBRARY_DIRS=",${BULLET_LIBRARY_DIRS})
# Include directories for GLFW, GLAD, and source files
include_directories(
    ${GLFW_DIR}/include
    ${GLAD_DIR}/include
    ${SRC_DIR}
    ${SDL_DIR}/include
    ${BULLET_INCLUDE_DIR}
)

# Add GLFW as a subdirectory (if you want to build GLFW as part of the project)
add_subdirectory(${GLFW_DIR})
add_subdirectory(${SDL_DIR}/SDL EXCLUDE_FROM_ALL)

# Add Glad source file
set(GLAD_SOURCES ${GLAD_DIR}/src/glad.c)

# Automatically collect all .cpp files inside src/
file(GLOB SOURCE_FILES "${SRC_DIR}/*.cpp")

# Iterate over each .cpp file and create an executable
foreach(SRC_FILE ${SOURCE_FILES})
    # Extract filename without extension
    get_filename_component(EXECUTABLE_NAME ${SRC_FILE} NAME_WE)

    # Create an executable for each .cpp file
    add_executable(${EXECUTABLE_NAME} ${SRC_FILE} ${GLAD_SOURCES})

    # Link GLFW and OpenGL
    find_package(OpenGL REQUIRED)
    target_link_libraries(${EXECUTABLE_NAME} PUBLIC OpenGL::GL)
    target_link_libraries(${EXECUTABLE_NAME} PUBLIC glfw)
    target_link_libraries(${EXECUTABLE_NAME} PUBLIC SDL3::SDL3)
    target_include_directories(${EXECUTABLE_NAME}  PUBLIC ${BULLET_INCLUDE_DIR})
    target_compile_definitions(${EXECUTABLE_NAME}  PUBLIC ${BULLET_DEFINITIONS})
    target_link_directories(${EXECUTABLE_NAME}  PUBLIC ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
    target_link_libraries(${EXECUTABLE_NAME}  PUBLIC BulletDynamics BulletCollision LinearMath)
    target_link_directories(${EXECUTABLE_NAME} PUBLIC ${BULLET_LIBRARY_DIRS})
    target_link_libraries(${EXECUTABLE_NAME} PUBLIC 
    ${BULLET_LIBRARY_DIRS}/libBulletDynamics.a
    ${BULLET_LIBRARY_DIRS}/libBulletCollision.a
    ${BULLET_LIBRARY_DIRS}/libLinearMath.a)


    # Set platform-specific OpenGL linking
    if(WIN32)
        target_link_libraries(${EXECUTABLE_NAME} PUBLIC opengl32)
    elseif(APPLE)
        target_link_libraries(${EXECUTABLE_NAME} PUBLIC OpenGL)
    else()
        target_link_libraries(${EXECUTABLE_NAME} PUBLIC GL)
    endif()
endforeach()

# Specify the CMake binary directory for build output
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
