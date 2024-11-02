cmake_minimum_required(VERSION 3.16)
project(log2rtcm)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=7 -DNEXOBS=3)
include_directories(./)
include_directories(../../../src)
include_directories(../../../src/rcv)
add_executable(log2rtcm
    log2rtcm.cpp
    ../../../src/rtklib.h   
    ../../../src/rtkcmn.c 
    ../../../src/rtcm.c 
    ../../../src/rtcm2.c 
    ../../../src/rtcm3.c 
    ../../../src/rtcm3e.c 
    ../../../src/preceph.c 
    ../../../src/sbas.c 
    ../../../src/ephemeris.c 
    ../../../src/rcvraw.c 
    ../../../src/rinex.c
    ../../../src/ionex.c
    ../../../src/convrnx.c
    ../../../src/pntpos.c
    ../../../src/geoid.c
    ../../../src/rcv/binex.c 
    ../../../src/rcv/crescent.c 
    ../../../src/rcv/javad.c 
    ../../../src/rcv/novatel.c 
    ../../../src/rcv/nvs.c 
    ../../../src/rcv/rt17.c 
    ../../../src/rcv/septentrio.c 
    ../../../src/rcv/skytraq.c 
    ../../../src/rcv/ss2.c 
    ../../../src/rcv/ublox.c
    )

target_link_libraries(log2rtcm m)
