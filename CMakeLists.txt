cmake_minimum_required(VERSION 3.23)
include(pico_sdk_import.cmake)
project(pmu11x_ctrl C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

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

add_compile_definitions(MG_ARCH=MG_ARCH_FREERTOS)

# ====== begin nanopb ======
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${NANOPB_SRC_ROOT_FOLDER}/extra)
find_package(Nanopb REQUIRED)
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS proto/messages.proto)
add_library(nanopb STATIC ${PROTO_SRCS})
target_include_directories(nanopb PUBLIC ${NANOPB_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
# ====== end nanopb ======

# ====== begin lfs ======
add_library(lfs STATIC littlefs/lfs.c littlefs/lfs_util.c)
target_include_directories(lfs PUBLIC littlefs)
target_link_libraries(lfs FreeRTOS)
target_compile_definitions(lfs PUBLIC LFS_THREADSAFE)
# ====== end lfs ======

add_compile_definitions(PICO_FLASH_SIZE_BYTES=16777216 MG_ENABLE_MIP errno=666)
add_compile_options(-fsingle-precision-constant -Werror=return-type -Wall)

include_directories(inc)

add_executable(pmu11x_ctrl main.cpp w5500.c rtt/SEGGER_RTT.c rtt/SEGGER_RTT_ASM_ARMv7M.S rtt/SEGGER_RTT_printf.c
        ws/mongoose.c controller/battery.cpp led.cpp controller/controller.cpp time/ds1302.cpp fs.cpp can2040/src/can2040.c controller/rectifier.cpp controller/config.cpp controller/http.cpp time/sntp.cpp)

pico_generate_pio_header(pmu11x_ctrl ${PROJECT_SOURCE_DIR}/time/ds1302.pio)

target_link_libraries(pmu11x_ctrl pico_stdlib hardware_dma hardware_spi pico_unique_id hardware_uart FreeRTOS nanopb hardware_pio hardware_flash hardware_adc lfs)