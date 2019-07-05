
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

set(CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    -DYAML_CPP_BUILD_TOOLS:BOOL=OFF
    -DBUILD_SHARED_LIBS:BOOL=OFF )

LIST(APPEND CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE})

ExternalProject_Add(yaml-cpp 
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp"
    PREFIX yaml-cpp
    CMAKE_ARGS ${CMAKE_ARGS})

ExternalProject_Get_Property(yaml-cpp install_dir)

add_library(yaml-cpp-lib STATIC IMPORTED GLOBAL)
set_target_properties(yaml-cpp-lib PROPERTIES IMPORTED_LOCATION
    "${install_dir}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}")

add_dependencies(yaml-cpp-lib yaml-cpp)

set(YAML_CPP_LIBRARIES "yaml-cpp-lib" CACHE INTERNAL "")
set(YAML_CPP_INCLUDE_DIR "${install_dir}/include" CACHE INTERNAL "")
