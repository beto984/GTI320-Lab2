cmake_minimum_required(VERSION 3.15)

project(labo02)

# Setup language requirements
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#--------------------------------------------------
# Sous MAC, OpenGL est Deprecated, mais toujours
# fonctionnel, on veut éviter tous les warnings
# durant la compilation.
#--------------------------------------------------
if (APPLE)
  add_definitions( -DGL_SILENCE_DEPRECATION )
endif()

#--------------------------------------------------
# Add nanogui and setup build
#--------------------------------------------------
FetchContent_Declare(
  nanogui
  GIT_REPOSITORY https://github.com/wjakob/nanogui.git
  GIT_TAG        e9ec8a1a9861cf578d9c6e85a6420080aa715c03
  GIT_PROGRESS TRUE
)

set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(NANOGUI_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(NANOGUI_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(NANOGUI_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(nanogui)
FetchContent_GetProperties(nanogui SOURCE_DIR nanogui_SRC_DIR BINARY_DIR nanogui_BIN_DIR)

add_definitions(${NANOGUI_EXTRA_DEFS})
include_directories(${nanogui_SRC_DIR}/include)
include_directories(${NANOGUI_EXTRA_INCS})
include_directories(${PROJECT_SOURCE_DIR}/../labo01/src ${COMMON_INCLUDES})

find_package(OpenGL REQUIRED)

# Add .cpp and .h files
set(HEADERS Icp.h IcpApplication.h IcpGLCanvas.h SVD.h)
set(SOURCE main.cpp Icp.cpp IcpApplication.cpp IcpGLCanvas.cpp)
add_executable(labo02 ${SOURCE} ${HEADERS})

target_link_libraries(labo02 nanogui opengl32 ${NANOGUI_EXTRA_LIBS})

if(MSVC) 
	set_property(TARGET labo02 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/labo02)
endif()