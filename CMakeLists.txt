cmake_minimum_required(VERSION 3.15)
project(gnss_positioning VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(gnss_positioning_lib
    src/NavMath.cpp
    src/NovatelDecoder.cpp
    src/SPP.cpp
    src/SatPosCalc.cpp
)
# public include paths for library; also expose third_party/eigen for users
target_include_directories(gnss_positioning_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen>
        $<INSTALL_INTERFACE:include>
)

add_executable(gnss_positioning
    src/main.cpp
)
# ensure executable can find eigen headers (library already provides if needed)
target_include_directories(gnss_positioning
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen
)
target_link_libraries(gnss_positioning
    PRIVATE gnss_positioning_lib
)

set_target_properties(gnss_positioning PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(gnss_positioning_lib PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

include(GNUInstallDirs)
install(TARGETS gnss_positioning_lib
    EXPORT gnss_positioningTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS gnss_positioning
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT gnss_positioningTargets
    NAMESPACE gnss_positioning::
    FILE gnss_positioningTargets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gnss_positioning
)
