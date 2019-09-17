cmake_minimum_required(VERSION 3.1)

project(skeleton_extraction)

### automatically download submodules
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/../.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()


add_subdirectory(external_library)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# matplotlib support
find_package(PythonLibs 2.7)

# specific version of eigen needed
find_package(Eigen3 3.2.10 EXACT REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# libigl
#option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
#option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
#option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
#option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
#option(LIBIGL_USE_STATIC_LIBRARY     "Use LibIGL as static library" ON)
#find_package(LIBIGL REQUIRED QUIET)

#find_package(OpenGL REQUIRED)

find_package(yaml-cpp REQUIRED)

file(GLOB SRCFILES app/*.cpp)

add_executable(${PROJECT_NAME}_bin ${SRCFILES})

set_target_properties(${PROJECT_NAME}_bin PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)

include_directories(${PROJECT_NAME}_bin
			        ${PROJECT_SOURCE_DIR}/external_library/
			        ${YAML_CPP_INCLUDE_DIR}
					${EIGEN3_INCLUDE_DIR}
					${PYTHON_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_bin
					  yaml-cpp
					  polyscope
					  ${PYTHON_LIBRARIES})

# add_dependencies(${PROJECT_NAME}_bin libGraphpp) # not linked as it is headers only
