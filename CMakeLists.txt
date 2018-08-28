cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME rrtstar)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/botlcm.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build(C_AGGREGATE_HEADER rrtstar.h)

pkg_check_modules(GLIB2 REQUIRED glib-2.0)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(BOT2_CORE REQUIRED bot2-core)

include_directories(${LCMTYPES_INCLUDE_DIRS})

pods_install_pkg_config_file(lcmtypes-rrtstar
    LIBS -l${LCMTYPES_LIBS}
    REQUIRES lcm
    VERSION 0.0.1)

add_subdirectory(src)
add_subdirectory(src/renderer)
