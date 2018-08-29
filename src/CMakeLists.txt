add_definitions(-std=gnu99)

add_executable(rrtstar
    main.c
    opttree.h
    optsystem.h
    optsystem.c
    opttree.c
    kdtree.h
    kdtree.c)

target_link_libraries (rrtstar
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(rrtstar
    lcm
    glib-2.0
    bot2-core
    lcmtypes_hr-lcmtypes
    lcmtypes_obstacle-detector
    bot2-param-client
    geom-utils
    check_gridmap)

# make exeutables public
pods_install_executables(rrtstar)



add_executable(testrrtstar
    test_rrtstar.c)

target_link_libraries (testrrtstar
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(testrrtstar
    lcm
    glib-2.0
    bot2-core
    lcmtypes_hr-lcmtypes)

# make exeutables public
pods_install_executables(testrrtstar)
