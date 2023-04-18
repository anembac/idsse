cmake_minimum_required(VERSION 3.12)

# read version information from external file
file(STRINGS VERSION _VERSION)

project(idsse
    LANGUAGES CXX
    VERSION ${_VERSION}
)

# add the folder 'cmake' to the available modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# include setup from conan if the file is there
if(EXISTS ${PROJECT_BINARY_DIR}/conanbuildinfo.cmake)
  include(${PROJECT_BINARY_DIR}/conanbuildinfo.cmake)
  conan_basic_setup()
endif()

#
# Set build type
#

# Build release version by default
if(NOT CMAKE_BUILD_TYPE)
    # note: CMAKE_BUILD_TYPE is always defined, if not provided it is an empty string
    set(CMAKE_BUILD_TYPE Release)
endif()

message("Building ${PROJECT_NAME} ${PROJECT_VERSION} (${CMAKE_BUILD_TYPE})")

#
# Some options to control aspects of the build process
#
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_DOC "Build documentation" OFF)
option(CHECK_DOC_STRICT "Check the documentation for missing entries" ON)

#
# Check for project dependencies
#

find_package(Boost 1.67 REQUIRED system filesystem thread)

# we need to find the threading library preferring pthread
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# Google ProtocolBuffers are used as the main message format internally
find_package(Protobuf 3.0.0 REQUIRED)

# add the protobuf include for the google proto files
list(APPEND PROTOBUF_IMPORT_DIRS ${PROTOBUF_INCLUDE_DIRS})

# find ezCar2X
find_package(ezC2X REQUIRED)

#
# Specify the default compiler warnings used for building
#
set(DEFAULT_STRICT_COMPILER_WARNINGS
  # clang/GCC warnings
  # note: -Wextra is not added due to issues with the generated protobuf files
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:-Werror;-pedantic-errors;-Wall;-Wconversion;-Wsign-conversion>
)

#
# More CMake specific stuff
#

# we want to use the default GNU install paths
include(GNUInstallDirs)

# set the directory for installing plugins
set(PLUGIN_DIR "${CMAKE_INSTALL_DATADIR}/ezC2X/plugins")


# add the main source
add_subdirectory(src)

# add testing
if(BUILD_TESTS)
  # include testing framework library, e.g. catch2
  # add_subdirectory(3rdparty/catch)
  enable_testing()

  # enable individual library tests
  # add_subdirectory(tests)
endif()

# Add documentation
if(BUILD_DOC)
  add_subdirectory(doc)
endif()


