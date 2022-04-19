cmake_minimum_required(VERSION 3.14)
project(rendevous)

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 17)

# Enable Compiler warnings
if(MSVC)
  add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wno-unused-parameter)
endif()

# SWIG generates some code that this warning isn't happy with
#add_compile_options(-Wno-missing-field-initializers)
# Make SWIG happy (something to do with dynamic linking)
#add_compile_options(-fPIC)
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-03")

# Set debug flags. These don't enable the debug flags, they just tell cmake
# which flags to set in debug mode.
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

################################# Dependencies ################################
include(FetchContent)

######## SymEngine ########
# TODO(ianruh): Make this optional so solvers can be built without needing
# symengine installed.
FetchContent_Declare(
    symengine
    GIT_REPOSITORY https://github.com/symengine/symengine.git
    GIT_TAG        v0.9.0
)
FetchContent_MakeAvailable(symengine)
include_directories(SYSTEM ${CMAKE_BINARY_DIR}/_deps/symengine-build/ ${symengine_SOURCE_DIR})
set(BUILD_TESTS OFF CACHE INTERNAL "SymEngine Test Building")
set(BUILD_BENCHMARKS OFF CACHE INTERNAL "SymEngine Benchmark Building")

# I don't know why this is needed. Shouldn't gmp be linked when SymEngine
# builds since it is building static libs?
FIND_LIBRARY(gmp NAMES gmp libgmp)

###### cppmpc ######
FetchContent_Declare(
    cppmpc
    GIT_REPOSITORY git@github.com:ianruh/cppmpc.git
)
FetchContent_MakeAvailable(cppmpc)
include_directories(SYSTEM ${CMAKE_BINARY_DIR}/_deps/cppmpc-src)
target_compile_options(cppmpc PRIVATE -Wno-cast-function-type)

######## Google Test ########
# Fetch GoogleTest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

##### matplot++ #########
FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG v1.1.0)
FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

####### Eigen #######
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

####### Boost ########
find_package(Boost 1.70 REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

####### ArgParse ########
FetchContent_Declare(
  argparse
  GIT_REPOSITORY https://github.com/p-ranav/argparse.git
  GIT_TAG 1a552dfd60cb8a478f7d7cb63e4814ac66ca2891
)
FetchContent_MakeAvailable(argparse)

################################# Targets #####################################

######## Libraries ########

# simulator Library
add_library(simulator
    src/Simulator.cpp
    src/Simulator.h
    src/CodeGen.cpp
    src/CodeGen.h
    src/Constants.h
    src/OrbitalState.h
    src/OrbitalState.cpp
    src/FiniteLQR.cpp
    src/FiniteLQR.h
    src/InfiniteLQR.h
    src/InfiniteLQR.cpp
    src/InfiniteLQRLinearTracking.h
    src/InfiniteLQRLinearTracking.cpp
    src/InfiniteLQRNonLinearTracking.h
    src/InfiniteLQRNonLinearTracking.cpp
    src/MPCNonLinearTracking.h
    src/MPCNonLinearTracking.cpp
    src/RiccatiSolvers.h
    src/RiccatiSolvers.cpp
    src/TrajectoryGeneration.h
    src/TrajectoryGeneration.cpp
    src/external/rapidcsv.h
)
target_include_directories(simulator PRIVATE ${CMAKE_BINARY_DIR}/_deps/cppmpc-src PUBLIC src/)
target_link_libraries(simulator Eigen3::Eigen matplot cppmpc symengine ${Boost_LIBRARIES})
target_compile_options(simulator PUBLIC -Wall -Wextra -Werror)

######## Executables ########

add_executable(rendevousMain
    src/main.cpp
    src/SimulationRuns.h
    src/SimulationRuns.cpp
)
target_link_libraries(rendevousMain PRIVATE
    simulator
    Eigen3::Eigen
    matplot
    argparse)
target_compile_options(rendevousMain PUBLIC -Wall -Wextra -Werror)

######## Tests ########
enable_testing()

add_executable(SimulatorTests
    tests/OrbitalStateTest.cpp
)
target_link_libraries(SimulatorTests simulator gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(SimulatorTests)
