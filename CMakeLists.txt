cmake_minimum_required(VERSION 3.23)
include(pico_sdk_import.cmake)
project(pmu11x_ctrl C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror=return-type")

add_library(FreeRTOS STATIC
        FreeRTOS/event_groups.c
        FreeRTOS/list.c
        FreeRTOS/queue.c
        FreeRTOS/stream_buffer.c
        FreeRTOS/tasks.c
        FreeRTOS/timers.c
        FreeRTOS/portable/GCC/ARM_CM0/port.c
        FreeRTOS/portable/MemMang/heap_4.c)

target_include_directories(FreeRTOS PUBLIC
        config
        FreeRTOS/portable/GCC/ARM_CM0
        FreeRTOS/include)

add_library(FreeRTOS_TCP STATIC
        FreeRTOS-Plus-TCP/source/FreeRTOS_ARP.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_DHCP.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_DNS.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Cache.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Callback.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Networking.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Parser.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_ICMP.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_IP.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_IP_Timers.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_IP_Utils.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_Sockets.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_Stream_Buffer.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_IP.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Reception.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_State_Handling.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Transmission.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Utils.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_WIN.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_Tiny_TCP.c
        FreeRTOS-Plus-TCP/source/FreeRTOS_UDP_IP.c
        FreeRTOS-Plus-TCP/source/portable/BufferManagement/BufferAllocation_2.c
        )

add_compile_definitions(MG_ARCH=MG_ARCH_FREERTOS MG_ENABLE_FREERTOS_TCP)

target_include_directories(FreeRTOS_TCP PUBLIC
        config
        FreeRTOS-Plus-TCP/source/portable/Compiler/GCC
        FreeRTOS-Plus-TCP/source/include
        )

target_link_libraries(FreeRTOS_TCP FreeRTOS)

# ====== begin coreSNTP ======
include(coreSNTP/coreSntpFilePaths.cmake)
add_library(coreSNTP STATIC ${CORE_SNTP_SOURCES})
target_include_directories(coreSNTP PUBLIC ${CORE_SNTP_INCLUDE_PUBLIC_DIRS} config)
# ====== end coreSNTP ======

# ====== begin nanopb ======
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${NANOPB_SRC_ROOT_FOLDER}/extra)
find_package(Nanopb REQUIRED)
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS proto/battery.proto)
add_library(nanopb STATIC ${PROTO_SRCS})
target_include_directories(nanopb PUBLIC ${NANOPB_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
# ====== end nanopb ======


include_directories(inc)

add_executable(pmu11x_ctrl main.cpp w5500.c crc.c rtt/SEGGER_RTT.c rtt/SEGGER_RTT_ASM_ARMv7M.S rtt/SEGGER_RTT_printf.c
        ws/mongoose.c battery.cpp led.cpp controller.cpp time/time.cpp time/ds1302.cpp)

pico_generate_pio_header(pmu11x_ctrl ${PROJECT_SOURCE_DIR}/time/ds1302.pio)

target_link_libraries(pmu11x_ctrl pico_stdlib hardware_dma hardware_spi pico_unique_id hardware_uart FreeRTOS_TCP FreeRTOS coreSNTP nanopb hardware_pio)