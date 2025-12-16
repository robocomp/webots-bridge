# Sources set
LIST(APPEND SOURCES
    ../src/specificworker.cpp
)

# Headers set
LIST(APPEND HEADERS
    ../src/specificworker.h
    fixedsizedeque.h
)


LIST(APPEND LIBS ${LIBS})

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

# Disabling Pragma region warnings.
add_definitions(-g -Wall -fmax-errors=1 -fno-char8_t -Wno-unknown-pragmas)

# Linking Gazebo libraries
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
SET (LIBS ${LIBS} -L/usr/local/webots/lib/controller Controller CppController)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

