set(CMAKE_C_COMPILER clang)
set(CMAKE_C_COMPILER_TARGET avr)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_COMPILER_TARGET avr)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -mmcu=atmega32u4)
cmake_minimum_required(VERSION 3.0.0)
project(progmem_test CXX)
add_executable(progmem_test main.cpp)
set(ARDUINO_CORE_VERSION 1.8.3)
set(ARDUINO_CORE_INCLUDES $ENV{HOME}/.arduino15/packages/arduino/hardware/avr/${ARDUINO_CORE_VERSION}/cores/arduino)

include_directories(BEFORE ${ARDUINO_CORE_INCLUDES})
# I'll hard code this for now. we can play around with discovery later on
set(AVR_GCC_VERSION "7.3.0-atmel3.6.1-arduino7")
set(AVR_GCC_INCLUDES $ENV{HOME}/.arduino15/packages/arduino/tools/avr-gcc/${AVR_GCC_VERSION}/avr/include)

include_directories(AFTER ${AVR_GCC_INCLUDES})
set(MCU_VARIANT_PACKAGE pololu-a-star) # the package that defines the variant
set(MCU_VARIANT_NAME a-star328pb)
set(MCU_VARIANT_VERSION 4.0.2)

set(MCU_VARIANT_INCLUDES $ENV{HOME}/.arduino15/packages/${MCU_VARIANT_PACKAGE}/hardware/avr/${MCU_VARIANT_VERSION}/variants/${MCU_VARIANT_NAME})

include_directories(AFTER ${MCU_VARIANT_INCLUDES})
set(AVR_CPU_FREQUENCY 16000000L)
set(AVR_USB_MANUFACTURER "Pololu Corporation")
set(AVR_USB_PRODUCT "Pololu A-Star 32U4")
set(AVR_USB_PRODUCT_ID 0x2300)
set(AVR_USB_VENDOR_ID 0x1ffb)

add_definitions(-DF_CPU=${AVR_CPU_FREQUENCY} -DARDUINO=10607 -DARDUINO_AVR_A_STAR_32U4
  -DARDUINO_ARCH_AVR -DUSB_VID=${AVR_USB_VENDOR_ID} -DUSB_PID=${AVR_USB_PRODUCT_ID}
  -DUSB_MANUFACTURER=${AVR_USB_MANUFACTURER} -DUSB_PRODUCT=${AVR_USB_PRODUCT})
add_definitions("-D__progmem__=address_space(1)")

target_compile_options(progmem_test PUBLIC ${CMAKE_CXX_FLAGS} -flto)
