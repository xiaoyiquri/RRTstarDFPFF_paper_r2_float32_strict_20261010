cmake_minimum_required(VERSION 3.16)
project(UnicycleTimeoutDiagnosis LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Reuse a completed Release build; production sources/configs are not changed.
foreach(path RRTPFF_EXISTING_BUILD RRTPFF_EIGEN_SOURCE RRTPFF_YAML_SOURCE RRTPFF_YAML_LIBRARY)
    if(NOT DEFINED ${path})
        message(FATAL_ERROR "Specify ${path}")
    endif()
endforeach()
add_executable(unicycle_timeout unicycle_timeout.cpp)
target_include_directories(unicycle_timeout PRIVATE ../.. "${RRTPFF_EIGEN_SOURCE}" "${RRTPFF_YAML_SOURCE}/include")
target_compile_definitions(unicycle_timeout PRIVATE YAML_CPP_STATIC_DEFINE)
if(MSVC)
    target_compile_options(unicycle_timeout PRIVATE /fp:strict)
else()
    target_compile_options(unicycle_timeout PRIVATE -ffp-contract=off)
endif()
target_link_libraries(unicycle_timeout PRIVATE
    "${RRTPFF_EXISTING_BUILD}/${CMAKE_STATIC_LIBRARY_PREFIX}rrtpff_paper_data${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${RRTPFF_EXISTING_BUILD}/${CMAKE_STATIC_LIBRARY_PREFIX}rrtstar_df_pff${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${RRTPFF_YAML_LIBRARY}")
