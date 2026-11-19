cmake_minimum_required(VERSION 3.16)
project(oscill-scopehal-plugin CXX)

# The plugin is loaded into ngscopeclient's address space and uses libscopehal's C++ classes directly,
# so it MUST be compiled against the exact same scopehal-apps source tree (and build) as the ngscopeclient
# binary you run it with.
#
#   cmake -B build -DSCOPEHAL_APPS_DIR=$HOME/scopehal-apps -DSCOPEHAL_BUILD_DIR=$HOME/scopehal-apps/build
#
set(SCOPEHAL_APPS_DIR "" CACHE PATH "Path to the scopehal-apps source checkout ngscopeclient was built from")
set(SCOPEHAL_BUILD_DIR "${SCOPEHAL_APPS_DIR}/build" CACHE PATH "Path to the scopehal-apps build directory")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# --- Protocol unit tests: no scopehal dependency ----------------------------------------------------------------------
option(OSCILL_BUILD_TESTS "Build protocol unit tests" ON)
if(OSCILL_BUILD_TESTS)
	enable_testing()
	add_executable(test_protocol tests/test_protocol.cpp src/OscillProtocol.cpp)
	target_include_directories(test_protocol PRIVATE src)
	add_test(NAME protocol COMMAND test_protocol)
endif()

if(NOT SCOPEHAL_APPS_DIR)
	message(WARNING "SCOPEHAL_APPS_DIR not set: only building protocol tests, not the plugin")
	return()
endif()

set(SCOPEHAL_LIB_DIR "${SCOPEHAL_APPS_DIR}/lib")
if(NOT EXISTS "${SCOPEHAL_LIB_DIR}/scopehal/scopehal.h")
	message(FATAL_ERROR "${SCOPEHAL_LIB_DIR}/scopehal/scopehal.h not found - is SCOPEHAL_APPS_DIR correct and are submodules checked out?")
endif()
if(NOT EXISTS "${SCOPEHAL_BUILD_DIR}/lib/scopehal/config.h")
	message(FATAL_ERROR "${SCOPEHAL_BUILD_DIR}/lib/scopehal/config.h not found - build scopehal-apps first or set SCOPEHAL_BUILD_DIR")
endif()

# --- Same third-party headers libscopehal itself uses ------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Vulkan REQUIRED)
pkg_check_modules(SIGCXX QUIET IMPORTED_TARGET sigc++-3.0)
if(NOT SIGCXX_FOUND)
	pkg_check_modules(SIGCXX REQUIRED IMPORTED_TARGET sigc++-2.0)
endif()
find_path(GLSLANG_INCLUDE_DIR glslang/Public/ShaderLang.h)

add_library(oscill SHARED
	src/OscillPlugin.cpp
	src/OscillOscilloscope.cpp
	src/OscillSerial.cpp
	src/OscillProtocol.cpp)

target_include_directories(oscill PRIVATE
	src
	${SCOPEHAL_APPS_DIR}/cmake
	${SCOPEHAL_LIB_DIR}
	${SCOPEHAL_LIB_DIR}/scopehal
	${SCOPEHAL_LIB_DIR}/log
	${SCOPEHAL_LIB_DIR}/xptools
	${SCOPEHAL_BUILD_DIR}/lib/scopehal)
target_include_directories(oscill SYSTEM PRIVATE
	${SCOPEHAL_LIB_DIR}/VkFFT/vkFFT
	${GLSLANG_INCLUDE_DIR}/glslang/Include
	${GLSLANG_INCLUDE_DIR}/glslang/Public)

# Must match libscopehal's compile definitions or class layouts may differ
target_compile_definitions(oscill PRIVATE VK_ENABLE_BETA_EXTENSIONS=1)
if(EXISTS "${SCOPEHAL_BUILD_DIR}/lib/scopehal/CMakeFiles/scopehal.dir/flags.make")
	file(READ "${SCOPEHAL_BUILD_DIR}/lib/scopehal/CMakeFiles/scopehal.dir/flags.make" _scopehal_flags)
	if(_scopehal_flags MATCHES "-DHAS_LXI")
		target_compile_definitions(oscill PRIVATE HAS_LXI)
	endif()
endif()

target_link_libraries(oscill PRIVATE yaml-cpp Vulkan::Vulkan PkgConfig::SIGCXX)

# Symbols from libscopehal / liblog / libxptools are resolved at dlopen() time from the already loaded ngscopeclient
# process, so we don't hard-link them (that keeps the plugin independent of where libscopehal.so is installed).
set_target_properties(oscill PROPERTIES PREFIX "lib" OUTPUT_NAME "oscill")
target_compile_options(oscill PRIVATE -Wall -Wextra -fvisibility=hidden)

install(TARGETS oscill LIBRARY DESTINATION "$ENV{HOME}/.scopehal/plugins")

# --- Optional end-to-end smoke test against a scopehal-apps *build tree* ----------------------------------------------
option(OSCILL_BUILD_SMOKE_TEST "Build tests/smoke_test (links libscopehal.so from SCOPEHAL_BUILD_DIR)" OFF)
if(OSCILL_BUILD_SMOKE_TEST)
	find_library(SCOPEHAL_LIBRARY scopehal PATHS "${SCOPEHAL_BUILD_DIR}/lib/scopehal" NO_DEFAULT_PATH REQUIRED)
	find_library(SCOPEHAL_LOG_LIBRARY log PATHS "${SCOPEHAL_BUILD_DIR}/lib/log" NO_DEFAULT_PATH REQUIRED)
	add_executable(smoke_test tests/smoke_test.cpp)
	get_target_property(_incs oscill INCLUDE_DIRECTORIES)
	target_include_directories(smoke_test PRIVATE ${_incs})
	target_include_directories(smoke_test SYSTEM PRIVATE
		${SCOPEHAL_LIB_DIR}/VkFFT/vkFFT
		${GLSLANG_INCLUDE_DIR}/glslang/Include
		${GLSLANG_INCLUDE_DIR}/glslang/Public)
	get_target_property(_defs oscill COMPILE_DEFINITIONS)
	target_compile_definitions(smoke_test PRIVATE ${_defs})
	target_link_libraries(smoke_test PRIVATE ${SCOPEHAL_LIBRARY} ${SCOPEHAL_LOG_LIBRARY} yaml-cpp Vulkan::Vulkan PkgConfig::SIGCXX)
	set_target_properties(smoke_test PROPERTIES ENABLE_EXPORTS ON)
endif()
