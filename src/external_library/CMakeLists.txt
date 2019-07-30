include(ExternalProject)

# yaml-cpp stuff
set(CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    -DYAML_CPP_BUILD_TOOLS:BOOL=OFF
    -DBUILD_SHARED_LIBS:BOOL=OFF )

LIST(APPEND CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE})

ExternalProject_Add(yaml-cpp 
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
    PREFIX yaml-cpp
    CMAKE_ARGS ${CMAKE_ARGS}
)

ExternalProject_Get_Property(yaml-cpp install_dir)

add_library(yaml-cpp-lib STATIC IMPORTED GLOBAL)
set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION
    "${install_dir}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}")

add_dependencies(yaml-cpp-lib yaml-cpp)

set(YAML_CPP_LIBRARIES "yaml-cpp-lib" CACHE INTERNAL "")
set(YAML_CPP_INCLUDE_DIR "${install_dir}/include" CACHE INTERNAL "")

# libgraphcpp
# import of headers only: https://stackoverflow.com/a/21223763/2562693
set(libgraphcpp_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/libgraphcpp")
set(libgraphcpp_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/libgraphcpp")
set(libgraphcpp_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${libgraphcpp_INSTALL_DIR})

message("libgraphcpp_PREFIX='${libgraphcpp_PREFIX}'")
message("libgraphcpp_INSTALL_DIR='${libgraphcpp_INSTALL_DIR}'")
message("libgraphcpp_CMAKE_ARGS='${libgraphcpp_CMAKE_ARGS}'")
message("GIT_EXECUTABLE='${GIT_EXECUTABLE}'")

ExternalProject_Add(libgraphcpp
  PREFIX libgraphcpp
  GIT_TAG master
  GIT_REPOSITORY https://github.com/rFalque/libGraphCpp
  CMAKE_ARGS ${libgraphcpp_CMAKE_ARGS}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
                    <SOURCE_DIR>/include ${CMAKE_BINARY_DIR}/external_library/libgraphcpp/include
)

ExternalProject_Get_Property(libgraphcpp install_dir)
set(LIB_GRAPH_CPP_INCLUDE_DIR "${install_dir}/include" CACHE INTERNAL "")
