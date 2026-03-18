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

add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
    -fomit-frame-pointer
    -funroll-loops
)
add_definitions(-DNDEBUG -fmax-errors=1 -Wno-unknown-pragmas)
add_link_options(-flto=auto)

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

# Linking Gazebo libraries
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
SET (LIBS ${LIBS} -L/usr/local/webots/lib/controller Controller CppController)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

