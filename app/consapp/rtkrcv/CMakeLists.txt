cmake_minimum_required(VERSION 3.16)
project(rtkrcv)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=7 -DNEXOBS=3)
include_directories(./)
include_directories(../../../src)
include_directories(../../../src/rcv)
add_executable(rtkrcv
    rtkrcv.c
    vt.h
    vt.c
    ../../../src/stream.c   
    ../../../src/rtkcmn.c   
    ../../../src/rtksvr.c   
    ../../../src/rtkpos.c   
    ../../../src/solution.c 
    ../../../src/lambda.c   
    ../../../src/sbas.c     
    ../../../src/rcvraw.c   
    ../../../src/rtcm.c     
    ../../../src/rtcm2.c    
    ../../../src/rtcm3.c    
    ../../../src/rtcm3e.c   
    ../../../src/preceph.c  
    ../../../src/options.c  
    ../../../src/pntpos.c   
    ../../../src/geoid.c
    ../../../src/ppp.c      
    ../../../src/ppp_ar.c   
    ../../../src/rcv/novatel.c  
    ../../../src/rcv/ublox.c    
    ../../../src/rcv/ss2.c      
    ../../../src/rcv/crescent.c 
    ../../../src/rcv/skytraq.c  
    ../../../src/rcv/javad.c    
    ../../../src/rcv/nvs.c      
    ../../../src/rcv/binex.c    
    ../../../src/rcv/rt17.c     
    ../../../src/rcv/septentrio.c
    ../../../src/ephemeris.c
    ../../../src/rinex.c    
    ../../../src/ionex.c    
    ../../../src/tides.c    
    )

find_package (Threads REQUIRED)    
target_link_libraries(rtkrcv m Threads::Threads)