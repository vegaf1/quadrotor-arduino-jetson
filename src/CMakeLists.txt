cmake_minimum_required(VERSION "3.11")

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
project("quadrotorcom")


#"$(PROJECT_NAME)"

set(CMAKE_CXX_STANDARD 14)
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS filesystem regex)
 

find_package(cppzmq)
#find_package(Threads)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

#include_directories(${Protobuf_INCLUDE_DIRS})

set(PROTO_MESSAGE_DIR /home/fausto/protobuff/c_gen/proto)

set(HEADERS ${PROTO_MESSAGE_DIR}/pwm.pb.h ${PROTO_MESSAGE_DIR}/pwm.pb.cc)


#add_library(libzmq STATIC IMPORTED)

add_executable(send /home/fausto/protobuff/src/send.cpp)

add_executable(receive /home/fausto/protobuff/src/receive.cpp)

add_executable(subscriber /home/fausto/protobuff/src/subscriber.cpp ${HEADERS})

#add_dependencies(send libzmq cppzmq)

#add_dependencies(receive libzmq cppzmq)

#target_include_directories(protobuff PUBLIC ${PROTOBUF_INCLUDE_DIRS} ${PROTO_MESSAGE_DIR})
add_library(protobuffers ${HEADERS})
target_include_directories(protobuffers PUBLIC include ${PROTO_MESSAGE_DIR})

target_link_libraries(send cppzmq)
target_link_libraries(receive cppzmq)
target_link_libraries(subscriber cppzmq ${PROTOBUF_LIBRARIES} protobuffers ${Boost_LIBRARIES})
#target_link_libraries(receive pthread)
#target_link_libraries( recieve ${CMAKE_THREAD_LIBS_INIT} )
target_include_directories(send PUBLIC ${PROTOBUF_INCLUDE_DIRS} ${PROTO_MESSAGE_DIR})
target_include_directories(receive PUBLIC /home/fausto/libzmq-master/include)
target_include_directories(subscriber PUBLIC /home/fausto/libzmq-master/include)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
    add_executable(serial /home/fausto/protobuff/src/serial.cpp) 
    target_link_libraries(serial ${Boost_LIBRARIES})
endif()


