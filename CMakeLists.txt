cmake_minimum_required( VERSION 3.6 )
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOUDINI_INSTALL_DIR}/toolkit")
project( SOP_developablePatcher )

# CMAKE_PREFIX_PATH must contain the path to the toolkit/cmake subdirectory of
# the Houdini installation. See the "Compiling with CMake" section of the HDK
# documentation for more details, which describes several options for
# specifying this path.
list( APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake" )

# Locate Houdini's libraries and header files.
# Registers an imported library target named 'Houdini'.
find_package( Houdini REQUIRED )

### START DEVELOPABILITY OF TRIANGLE MESHES ###
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(LIBIGL)

if (NOT LIBIGL_FOUND)
message(FATAL_ERROR "libigl not found --- You can download it using: \n git clone --recursive https://github.com/libigl/libigl.git ${PROJECT_SOURCE_DIR}/../libigl")
endif()

# Compilation flags: adapt to your needs
if(MSVC)
# Enable parallel compilation
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR} )
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR} )
else()
# Libigl requires a modern C++ compiler that supports c++11
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "." )
endif()

# Prepare the build environment
#TODO ACTIVATE THESE BACK ON
#include_directories(${LIBIGL_INCLUDE_DIRS})
#add_definitions(${LIBIGL_DEFINITIONS})

### END OF DEVELOPABILITY OF TRIANGLE MESHES ###

include_directories(${CMAKE_SOURCE_DIR}/external/eigen)
#include_directories(${CMAKE_SOURCE_DIR}/external/libigl/include)

set( library_name SOP_developablePatcher )

# Code generation for the embedded DS file in SOP_developablePatcher.C.
houdini_generate_proto_headers( FILES SOP_developablePatcher.C )

# Add a library and its source files.
add_library( ${library_name} SHARED
    SOP_developablePatcher.C
    SOP_developablePatcher.h
)

# Link against the Houdini libraries, and add required include directories and
# compile definitions.
target_link_libraries( ${library_name} Houdini )

# Include ${CMAKE_CURRENT_BINARY_DIR} for the generated header.
target_include_directories( ${library_name} PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Sets several common target properties, such as the library's output directory.
houdini_configure_target( ${library_name} )

