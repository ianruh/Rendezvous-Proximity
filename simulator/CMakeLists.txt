cmake_minimum_required(VERSION 3.14)
project(rendevous)

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 17)

# Enable Compiler warnings
if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
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

######## Google Test ########
# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

####### Eigen #######
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

################################# Targets #####################################

######## Libraries ########

# simulator Library
add_library(simulator
    src/Simulator.cpp
    src/Simulator.h
    src/Constants.h
    src/OrbitalState.h
    src/OrbitalState.cpp
)
target_include_directories(simulator PUBLIC src/)
target_link_libraries(simulator Eigen3::Eigen)

######## Executables ########

add_executable(MPCController src/MPCController.cpp)
target_link_libraries(MPCController PRIVATE simulator)

######## Tests ########
enable_testing()

add_executable(SimulatorTests
    tests/OrbitalStateTest.cpp
)
target_link_libraries(SimulatorTests simulator gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(SimulatorTests)
