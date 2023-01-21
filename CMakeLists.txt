cmake_minimum_required(VERSION 3.23)
include(pico_sdk_import.cmake)
project(pmu11x_ctrl C CXX ASM)

set(CMAKE_CXX_STANDARD 14)
pico_sdk_init()


add_library(FreeRTOS STATIC
        FreeRTOS/event_groups.c
        FreeRTOS/list.c
        FreeRTOS/queue.c
        FreeRTOS/stream_buffer.c
        FreeRTOS/tasks.c
        FreeRTOS/timers.c
        FreeRTOS/portable/MemMang/heap_4.c
        FreeRTOS/portable/GCC/ARM_CM0/port.c
        )

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

include_directories(inc ioLibrary_Driver/Ethernet)


add_executable(pmu11x_ctrl main.cpp w5500.c crc.c rtt/SEGGER_RTT.c rtt/SEGGER_RTT_ASM_ARMv7M.S rtt/SEGGER_RTT_printf.c
        ws/mongoose.c)

target_link_libraries(pmu11x_ctrl pico_stdlib FreeRTOS FreeRTOS_TCP hardware_dma hardware_spi pico_unique_id)