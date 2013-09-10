cmake_minimum_required(VERSION 2.6)

project(chiligeom)

set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "0")

find_package( OpenCV REQUIRED )

# documentation generation
include("${CMAKE_SOURCE_DIR}/cmake/TargetDoc.cmake" OPTIONAL)


##########################################
##           Library                   ###
##########################################


file(
	GLOB_RECURSE
	source
	src/*
)

file(
	GLOB_RECURSE
	headers
	include/*
)

add_library(
        chiligeom
        SHARED
        ${source}
)

include_directories(include)
target_link_libraries(chiligeom)

install (TARGETS chiligeom
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

install(FILES
	${headers}
	DESTINATION include/chiligeom
)



##########################################
##           Options                   ###
##########################################


option (COMPILE_TESTS "compile demo applications" ON)

if (COMPILE_TESTS)

    find_package(PkgConfig)

    pkg_search_module(CHILITAGS REQUIRED chilitags)

    file(
        GLOB_RECURSE
        tags_pose_src
        samples/tags_pose/*
    )
    include_directories(include ${CHILITAGS_INCLUDE_DIRS})
    add_executable(
        tags_pose
        ${tags_pose_src}
    )

target_link_libraries( tags_pose chiligeom ${CHILITAGS_LDFLAGS} ${OpenCV_LIBS} )

endif()

##########################################
##       PKGCONFIG file                ###
##########################################

if(EXISTS "${CMAKE_SOURCE_DIR}/chiligeom.pc.in")
    message(STATUS "configuring ${CMAKE_SOURCE_DIR}/chiligeom.pc.in --> ${CMAKE_CURRENT_BINARY_DIR}/chiligeom.pc")
    configure_file(${CMAKE_SOURCE_DIR}/chiligeom.pc.in 
      ${CMAKE_CURRENT_BINARY_DIR}/chiligeom.pc
      @ONLY )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/chiligeom.pc
        DESTINATION lib/pkgconfig
    )
else()
    message(STATUS "${CMAKE_SOURCE_DIR}/chiligeom.pc.in not found!")

endif()


