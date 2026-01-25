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

set (PROJECT_VERSION  "1.1.0")
project(MAVLINK_SDK_CPP VERSION ${PROJECT_VERSION})



SET (CMAKE_C_COMPILER /usr/bin/gcc)
SET (CMAKE_CXX_COMPILER /usr/bin/g++)

#C++ Version
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED True)





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
  $<$<CONFIG:DEBUG>:-g3>
  $<$<CONFIG:DEBUG>:-Og>
  $<$<CONFIG:RELEASE>:-O3>
  $<$<CONFIG:RELEASE>:-Werror=unused-variable>
  $<$<CONFIG:RELEASE>:-Werror=unused-result>
  $<$<CONFIG:RELEASE>:-Werror=parentheses>
)

# adding DEFINEs
add_compile_definitions(
    $<$<CONFIG:DEBUG>:DEBUG>
    $<$<CONFIG:RELEASE>:RELEASE>
)


SET(OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${OUTPUT_DIRECTORY})

file(GLOB main "./src/*.cpp")
file(GLOB folder_helpers "./src/helpers/*.cpp")

set(files ${main} ${folder_helpers})

include_directories (${PROJECT_SOURCE_DIR}/../c_library_v2)

#Create a static library
add_library(OUTPUT_BINARY STATIC ${files})

set_target_properties(OUTPUT_BINARY 
                PROPERTIES 
                    OUTPUT_NAME "mavlink_sdk"
                    SUFFIX ".a"
                )

add_library(OUTPUT_BINARY_DYN SHARED ${files})

set_target_properties(OUTPUT_BINARY_DYN 
                PROPERTIES 
                    OUTPUT_NAME "mavlink_sdk"
                    SUFFIX ".so"
                )
                               
add_custom_command(TARGET OUTPUT_BINARY POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OUTPUT_BINARY> ${OUTPUT_DIRECTORY}
    COMMENT "Created ${PROJECT_BINARY_DIR}/${OUTPUT_BINARY}"
    )
    
add_custom_command(TARGET OUTPUT_BINARY_DYN POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OUTPUT_BINARY_DYN> ${OUTPUT_DIRECTORY}
    COMMENT "Created ${PROJECT_BINARY_DIR}/${OUTPUT_BINARY_DYN}"
    )
message ("${Yellow}=========================================================================${ColourReset}")

