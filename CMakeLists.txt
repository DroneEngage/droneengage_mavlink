cmake_minimum_required(VERSION 3.1.0)

if(NOT WIN32)
  string(ASCII 27 Esc)
  SET(ColourReset "${Esc}[m")
  SET(ColourBold  "${Esc}[1m")
  SET(Red         "${Esc}[31m")
  SET(Green       "${Esc}[32m")
  SET(Yellow      "${Esc}[33m")
  SET(Blue        "${Esc}[34m")
  SET(Magenta     "${Esc}[35m")
  SET(Cyan        "${Esc}[36m")
  SET(White       "${Esc}[37m")
  SET(BoldRed     "${Esc}[1;31m")
  SET(BoldGreen   "${Esc}[1;32m")
  SET(BoldYellow  "${Esc}[1;33m")
  SET(BoldBlue    "${Esc}[1;34m")
  SET(BoldMagenta "${Esc}[1;35m")
  SET(BoldCyan    "${Esc}[1;36m")
  SET(BoldWhite   "${Esc}[1;37m")
endif()


if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
message ("${Red} In-source builds not allowed! Create a build directory and run CMake from there.  ${ColourReset}")
message( FATAL_ERROR “In-source builds not allowed! Create a build directory and run CMake from there. ” )
exit(1)
endif()

# Auto-increment version on each build
set(VERSION_FILE "${CMAKE_SOURCE_DIR}/.version")

# Manually set major, minor, and bugfix versions (you can change these)
set(MAJOR_VERSION 5)
set(MINOR_VERSION 6)
set(BUGFIX_VERSION 8)

if(EXISTS ${VERSION_FILE})
    file(READ ${VERSION_FILE} CURRENT_VERSION)
    string(STRIP "${CURRENT_VERSION}" CURRENT_VERSION)
    # Extract current version components
    string(REPLACE "." ";" VERSION_LIST ${CURRENT_VERSION})
    list(LENGTH VERSION_LIST LIST_LENGTH)
    if(LIST_LENGTH EQUAL 4)
        list(GET VERSION_LIST 0 STORED_MAJOR)
        list(GET VERSION_LIST 1 STORED_MINOR)
        list(GET VERSION_LIST 2 STORED_BUGFIX)
        list(GET VERSION_LIST 3 STORED_BUILD)
        
        # Check if major, minor, or bugfix version changed
        if(NOT (${STORED_MAJOR} EQUAL ${MAJOR_VERSION} AND ${STORED_MINOR} EQUAL ${MINOR_VERSION} AND ${STORED_BUGFIX} EQUAL ${BUGFIX_VERSION}))
            set(BUILD_VERSION 0)  # Reset build when major/minor/bugfix changes
            message(STATUS "Version changed from ${STORED_MAJOR}.${STORED_MINOR}.${STORED_BUGFIX}.${STORED_BUILD} to ${MAJOR_VERSION}.${MINOR_VERSION}.${BUGFIX_VERSION}, resetting build to 0")
        else()
            set(BUILD_VERSION ${STORED_BUILD})  # Continue with existing build
        endif()
    else()
        set(BUILD_VERSION 0)  # Default build version
    endif()
else()
    set(BUILD_VERSION 0)  # Default build version
endif()

# Increment build version only for RELEASE builds
IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    math(EXPR BUILD_VERSION "${BUILD_VERSION} + 1")
    message(STATUS "Incrementing build version for RELEASE build")
ELSE()
    message(STATUS "Non-release build - keeping current build version")
ENDIF()

# Create new version string
set(__APP__VERSION__ "${MAJOR_VERSION}.${MINOR_VERSION}.${BUGFIX_VERSION}.${BUILD_VERSION}")

message(STATUS "Building version: ${__APP__VERSION__}")
message(STATUS "To change major/minor/bugfix versions, edit MAJOR_VERSION, MINOR_VERSION, and BUGFIX_VERSION above")

SET (__APP__VERSION__ "${__APP__VERSION__}")
add_definitions( -D__APP__VERSION__="${__APP__VERSION__}")
project(DE_MAVLINK_PLUGIN VERSION "${__APP__VERSION__}")



SET (CMAKE_C_COMPILER /usr/bin/gcc)
SET (CMAKE_CXX_COMPILER /usr/bin/g++)

#C++ Version
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Threads REQUIRED)


# Define an option for user control over DETAILED_DEBUG
option(DDEBUG "Detailed Debug" OFF) # Default is OFF


#define default build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
SET(CMAKE_BUILD_TYPE DEBUG)
ENDIF()

message ("${Yellow}=========================================================================${ColourReset}")
message ("CMAKE_C_COMPILER: "  ${CMAKE_C_COMPILER})
message ("CMAKE_CXX_COMPILER: " ${CMAKE_CXX_COMPILER})

#deceide debug or release
IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
message("CMAKE_BUILD_TYPE: ${BoldYellow} DEBUG ${ColourReset}")
ELSEIF (CMAKE_BUILD_TYPE MATCHES RELEASE)
message("CMAKE_BUILD_TYPE: ${BoldGreen} RELEASE ${ColourReset}")
ENDIF()

# adding CMAKE_CXX_FLAGS properly
add_compile_options(
  -Wno-return-type 
  -Wno-address-of-packed-member
  -Werror=unused-variable
  -Werror=unused-result
  $<$<CONFIG:DEBUG>:-g3>
  $<$<CONFIG:DEBUG>:-Og>
  $<$<CONFIG:RELEASE>:-O2>
  $<$<CONFIG:RELEASE>:-Werror=parentheses>
)

# adding DEFINEs
add_compile_definitions(
    $<$<CONFIG:DEBUG>:DEBUG>
    $<$<CONFIG:RELEASE>:RELEASE>
)

# add_link_options(
#   $<$<CONFIG:DEBUG>:>
#   $<$<CONFIG:RELEASE>:>
# )

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-export-dynamic")


#add_subdirectory(src)

file(GLOB folder_sdk "./mavlink_sdk/src/*.cpp")
file(GLOB folder_sdk_helper "./mavlink_sdk/src/helpers/*.cpp")
file(GLOB folder_src "./src/*.cpp")
file(GLOB folder_helpers "./src/helpers/*.cpp")
file(GLOB folder_udpproxy "./src/udp_proxy/*.cpp")
file(GLOB folder_uavos "./src/de_common/*.cpp")
file(GLOB folder_uavos1 "./src/de_common/de_databus/*.cpp")
file(GLOB folder_uavos2 "./src/de_common/helpers/*.cpp")
file(GLOB folder_mission "./src/mission/*.cpp")
file(GLOB folder_geofence "./src/geofence/*.cpp")
file(GLOB folder_general_mission_planner "./src/de_general_mission_planner/*.cpp")
file(GLOB folder_swarm "./src/swarm/*.cpp")
file(GLOB folder_tracking "./src/tracking/*.cpp")



set(files ${folder_src} ${folder_udpproxy} ${folder_uavos} ${folder_uavos1} ${folder_uavos2} ${folder_helpers}
 ${folder_mission} ${folder_geofence}  ${folder_swarm}
 ${folder_sdk} ${folder_sdk_helper} ${folder_tracking} ${folder_general_mission_planner})

include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
include_directories (${PROJECT_SOURCE_DIR}/mavlink_sdk/src)
include_directories (${PROJECT_SOURCE_DIR}/c_library_v2)

add_executable( de_mavlink ${files})

set_target_properties( de_mavlink 
                PROPERTIES 
                    OUTPUT_NAME "de_ardupilot"
                )


target_link_libraries(de_mavlink  Threads::Threads)

# Conditionally define DDEBUG based on the DDEBUG option
target_compile_definitions(de_mavlink PRIVATE
    $<$<BOOL:${DDEBUG}>:DDEBUG>
)

SET(OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${OUTPUT_DIRECTORY})
SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_DIRECTORY})


configure_file(de_mavlink.config.module.json ${OUTPUT_DIRECTORY}/de_mavlink.config.module.json COPYONLY)
configure_file(template.json ${OUTPUT_DIRECTORY}/template.json COPYONLY)

# Highlight if DDEBUG or TEST_MODE_NO_HAILO_LINK are enabled
if (DDEBUG)
    message(STATUS "${Red}Option DDEBUG is ENABLED.${ColourReset}")
endif()

message ("${Yellow}=========================================================================${ColourReset}")


# CPack configuration for package generation without installation
set(CPACK_PACKAGE_NAME "de-mavlink-plugin")
set(CPACK_PACKAGE_VERSION "${__APP__VERSION__}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libstdc++6") # Example dependency
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "DroneEngage Team")
set(CPACK_DEBIAN_PACKAGE_SECTION "science")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "DroneEngage MAVLink Plugin - Ardupilot communication module")

# Configure CPack to generate packages in build directory
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}/packages")
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_INSTALL_PREFIX "/home/$USER/drone_engage/de_mavlink")

# Enable DEB generator
set(CPACK_GENERATOR "DEB;TGZ;STGZ")

include(CPack)

# Installation configuration
install(TARGETS de_mavlink
    RUNTIME DESTINATION .
    COMPONENT runtime
)

install(FILES 
    ${PROJECT_SOURCE_DIR}/de_mavlink.config.module.json
    ${PROJECT_SOURCE_DIR}/template.json
    DESTINATION .
    COMPONENT runtime
)

set(BUILD_SUCCESS_SCRIPT "${CMAKE_BINARY_DIR}/build_success.cmake")
file(WRITE "${BUILD_SUCCESS_SCRIPT}" "")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E echo \"\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --green --bold \"=========================================================================\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --green --bold \"BUILD COMPLETED SUCCESSFULLY\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --cyan --bold \"Version: ${__APP__VERSION__}\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --cyan --bold \"Build Number: ${BUILD_VERSION}\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --cyan --bold \"Dependency Files: ${CMAKE_BINARY_DIR}/CMakeFiles/de_mavlink.dir/*.d\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E cmake_echo_color --green --bold \"=========================================================================\")\n")
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E echo \"\")\n")

IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
file(APPEND "${BUILD_SUCCESS_SCRIPT}" "file(WRITE \"${VERSION_FILE}\" \"${__APP__VERSION__}\")\n")
ENDIF()

